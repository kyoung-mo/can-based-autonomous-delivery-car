c// main.cpp (최종본: 라인 + ArUco 동시, 라인 안정화(컨투어 선택+EMA) + PD 제어 + ArUco 히스테리시스 + ROI 디버그 창 보기좋게 고정)
//
// Build:
// g++ -std=c++17 -O2 main.cpp -o run `pkg-config --cflags --libs opencv4`
// ./run

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;

// =========================
// 기본 설정
// =========================
static const int FRAME_W = 640;
static const int FRAME_H = 480;

// ROI: 하단만 사용(마커/종이 영향 줄임)
static const double ROI_Y_START_RATIO = 0.80; // 하단 20%

// 라인 색 모드
enum LineMode { BLACK_ON_WHITE, WHITE_ON_BLACK };
static const LineMode LINE_MODE = BLACK_ON_WHITE;

// 라인 노이즈 컷
static const double MIN_LINE_AREA = 250.0;

// =========================
// 제어 파라미터 (실주행 튜닝 포인트)
// =========================
static const double KP = 0.85;
static const double KD = 0.35;

// EMA(지수평활) 필터
static const double CX_EMA_ALPHA = 0.30;
static const double D_EMA_ALPHA  = 0.30;

// 속도(표시/연결용)
static const double CRUISE_SPEED = 0.35;

// steer 제한
static const double STEER_LIMIT = 1.0;

// =========================
// ArUco 설정
// =========================
static const int ARUCO_DICT = cv::aruco::DICT_4X4_50;

// ArUco 안정화(히스테리시스)
static const int ARUCO_ON_N  = 4;
static const int ARUCO_OFF_N = 8;

// =========================
// 유틸
// =========================
static double clampDouble(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// =========================
// (여기에 실제 모터 제어/CAN/PWM 연결)
// =========================
static void setDrive(double steer, double speed) {
    // TODO: CAN payload / PWM 매핑해서 보내기
    // 지금은 테스트 단계라 출력/연결 생략
    (void)steer;
    (void)speed;
}

// =========================
// 라인 검출: 컨투어 선택 안정화(이전 cx 기반)
// =========================
static int g_prevCx = -1;

static int detectLineCenterXStable(const Mat& frameBgr, Mat& debugRoiView) {
    int y0 = (int)(frameBgr.rows * ROI_Y_START_RATIO);
    Rect roi(0, y0, frameBgr.cols, frameBgr.rows - y0);
    Mat roiBgr = frameBgr(roi);

    // 1) Gray
    Mat gray;
    cvtColor(roiBgr, gray, COLOR_BGR2GRAY);

    // 2) Blur
    GaussianBlur(gray, gray, Size(5, 5), 0);

    // 3) Adaptive threshold
    Mat bin;
    adaptiveThreshold(gray, bin, 255,
                      ADAPTIVE_THRESH_GAUSSIAN_C,
                      THRESH_BINARY,
                      31, 7);

    if (LINE_MODE == BLACK_ON_WHITE) bitwise_not(bin, bin);

    // 4) Morphology
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(bin, bin, MORPH_OPEN, kernel, Point(-1, -1), 1);
    morphologyEx(bin, bin, MORPH_CLOSE, kernel, Point(-1, -1), 2);

    // 5) Contours
    vector<vector<Point>> contours;
    findContours(bin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat vis;
    cvtColor(bin, vis, COLOR_GRAY2BGR);

    if (contours.empty()) {
        debugRoiView = vis;
        return -1;
    }

    // 점수 기반 선택
    int bestIdx = -1;
    double bestScore = -1e18;
    const int roiCenterX = bin.cols / 2;

    for (int i = 0; i < (int)contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area < MIN_LINE_AREA) continue;

        Moments m = moments(contours[i]);
        if (m.m00 == 0) continue;

        int cx = (int)(m.m10 / m.m00);
        int cy = (int)(m.m01 / m.m00);

        // score:
        //  - area 클수록 좋고
        //  - ROI 아래쪽(cy가 클수록) 좋고
        //  - 이전 중심(prevCx)에 가까울수록 좋음(없으면 중앙 근처)
        double score = 0.0;
        score += area * 0.5;
        score += cy * 60.0;

        if (g_prevCx >= 0) score -= abs(cx - g_prevCx) * 250.0;
        else               score -= abs(cx - roiCenterX) * 80.0;

        if (score > bestScore) {
            bestScore = score;
            bestIdx = i;
        }
    }

    if (bestIdx < 0) {
        debugRoiView = vis;
        return -1;
    }

    // 선택된 컨투어 중심
    Moments m = moments(contours[bestIdx]);
    int cx = (m.m00 != 0.0) ? (int)(m.m10 / m.m00) : -1;
    if (cx >= 0) g_prevCx = cx;

    // debug draw
    drawContours(vis, contours, bestIdx, Scalar(0, 255, 0), 2);
    if (cx >= 0) circle(vis, Point(cx, vis.rows / 2), 6, Scalar(0, 0, 255), -1);

    debugRoiView = vis;
    return cx;
}

// =========================
// ArUco 검출(raw) + 표시
// =========================
static void detectAruco4x4(
    const Mat& frameBgr,
    vector<int>& ids,
    vector<vector<Point2f>>& corners,
    Mat& debugView
) {
    Mat gray;
    cvtColor(frameBgr, gray, COLOR_BGR2GRAY);

    // 조명 보정
    Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8, 8));
    Mat eq;
    clahe->apply(gray, eq);

    GaussianBlur(eq, eq, Size(3, 3), 0);

    // Ptr 요구하는 aruco.hpp 환경 대응
    Ptr<aruco::Dictionary> dict =
        makePtr<aruco::Dictionary>(aruco::getPredefinedDictionary(ARUCO_DICT));
    Ptr<aruco::DetectorParameters> params = makePtr<aruco::DetectorParameters>();

    // 인식률 튜닝(작은 마커/조명 변화 대응)
    params->adaptiveThreshWinSizeMin = 3;
    params->adaptiveThreshWinSizeMax = 35;
    params->adaptiveThreshWinSizeStep = 10;
    params->adaptiveThreshConstant = 5;

    params->minMarkerPerimeterRate = 0.02;
    params->maxMarkerPerimeterRate = 4.0;

    params->minDistanceToBorder = 2;

    params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    params->cornerRefinementWinSize = 5;
    params->cornerRefinementMaxIterations = 30;
    params->cornerRefinementMinAccuracy = 0.1;

    aruco::detectMarkers(eq, dict, corners, ids, params);

    debugView = frameBgr.clone();
    if (!ids.empty()) {
        aruco::drawDetectedMarkers(debugView, corners, ids);
    }
}

int main() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "카메라를 열 수 없습니다.\n";
        return 1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, FRAME_W);
    cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_H);

    cout << "OpenCV: " << CV_VERSION << "\n";
    cout << "ESC to quit\n";

    // ===== 창 크기 보기 좋게 고정 =====
    namedWindow("main", WINDOW_NORMAL);
    resizeWindow("main", 900, 600);

    namedWindow("line_roi_debug", WINDOW_NORMAL);
    resizeWindow("line_roi_debug", 500, 250); // 보기 좋은 고정 크기

    // ArUco 안정화 상태
    bool arucoStable = false;
    int onStreak = 0, offStreak = 0;
    int stableId = -1;

    // 라인 중심 EMA
    double cxFiltered = -1.0;

    // PD: error, dError 필터
    double prevError = 0.0;
    double dFiltered = 0.0;

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        resize(frame, frame, Size(FRAME_W, FRAME_H));

        // 1) ArUco raw
        vector<int> ids;
        vector<vector<Point2f>> corners;
        Mat arucoDebug;
        detectAruco4x4(frame, ids, corners, arucoDebug);

        bool arucoNow = !ids.empty();
        int idNow = arucoNow ? ids[0] : -1;

        // 2) ArUco 히스테리시스
        if (arucoNow) { onStreak++; offStreak = 0; }
        else          { offStreak++; onStreak = 0; }

        if (!arucoStable && onStreak >= ARUCO_ON_N) {
            arucoStable = true;
            stableId = idNow;
        }
        if (arucoStable && offStreak >= ARUCO_OFF_N) {
            arucoStable = false;
            stableId = -1;
        }

        // 3) Line detection (안정 컨투어 선택)
        Mat lineDebugRoi;
        int lineCx = detectLineCenterXStable(frame, lineDebugRoi);
        bool lineFound = (lineCx >= 0);

        // 4) EMA로 lineCx 안정화
        int cxUse = lineCx;
        if (lineFound) {
            if (cxFiltered < 0) cxFiltered = lineCx;
            cxFiltered = (1.0 - CX_EMA_ALPHA) * cxFiltered + CX_EMA_ALPHA * (double)lineCx;
            cxUse = (int)llround(cxFiltered);
        } else {
            cxFiltered = -1.0;
        }

        // 5) PD 제어
        double steer = 0.0;
        if (lineFound) {
            double centerX = frame.cols / 2.0;
            double error = ((double)cxUse - centerX) / centerX; // -1~+1
            double dErr = error - prevError;

            dFiltered = (1.0 - D_EMA_ALPHA) * dFiltered + D_EMA_ALPHA * dErr;

            steer = KP * error + KD * dFiltered;
            steer = clampDouble(steer, -STEER_LIMIT, STEER_LIMIT);

            prevError = error;
        } else {
            steer = 0.0;
            prevError = 0.0;
            dFiltered = 0.0;
        }

        // (테스트 단계) 실제 모터 출력은 아직 연결 안 함
        setDrive(steer, CRUISE_SPEED);

        // 6) 시각화
        Mat vis = arucoDebug.clone();

        int y0 = (int)(FRAME_H * ROI_Y_START_RATIO);
        rectangle(vis, Rect(0, y0, FRAME_W, FRAME_H - y0), Scalar(255, 0, 0), 2);

        if (lineFound) {
            circle(vis, Point(cxUse, y0 + (FRAME_H - y0) / 2), 6, Scalar(0, 0, 255), -1);
        }

        char buf[256];
        snprintf(buf, sizeof(buf),
                 "arucoNow=%s(id=%d) stable=%s(id=%d) | line=%s(cx=%d->%d) | steer=%.2f",
                 (arucoNow ? "ON" : "OFF"), idNow,
                 (arucoStable ? "ON" : "OFF"), stableId,
                 (lineFound ? "ON" : "OFF"), lineCx, cxUse,
                 steer);

        putText(vis, buf, Point(10, 30),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);

        imshow("main", vis);

        // ROI 디버그: "크기 고정 창"에 그대로 띄움(확대/축소 없이)
        imshow("line_roi_debug", lineDebugRoi);

        int key = waitKey(1);
        if (key == 27) break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
