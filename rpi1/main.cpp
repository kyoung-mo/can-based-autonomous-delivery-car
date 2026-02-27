#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;

// =========================
// 설정/튜닝 파라미터
// =========================
static const int FRAME_W = 640;
static const int FRAME_H = 480;

// ROI: 하단만 라인 추적(안정 + 속도)
static const double ROI_Y_START_RATIO = 0.60; // 하단 40%

// 라인 색 모드
// - BLACK_ON_WHITE: 검은 테이프/라인 + 밝은 바닥(추천 기본)
// - WHITE_ON_BLACK: 흰 라인 + 어두운 바닥
enum LineMode { BLACK_ON_WHITE, WHITE_ON_BLACK };
static const LineMode LINE_MODE = BLACK_ON_WHITE;

// 제어 (간단 P)
static const double KP = 0.9;

// ArUco dictionary
static const int ARUCO_DICT = cv::aruco::DICT_4X4_50;

// 면적/노이즈 컷
static const double MIN_LINE_AREA = 200.0;

// =========================
// clamp
// =========================
static double clampDouble(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// =========================
// 라인 중심 검출 (ROI 기준 x 반환, 없으면 -1)
// debugRoiView: 디버그 시각화 이미지(ROI 영역)
// =========================
static int detectLineCenterX(const Mat& frameBgr, Mat& debugRoiView) {
    int y0 = (int)(frameBgr.rows * ROI_Y_START_RATIO);
    Rect roi(0, y0, frameBgr.cols, frameBgr.rows - y0);
    Mat roiBgr = frameBgr(roi);

    // 1) Gray
    Mat gray;
    cvtColor(roiBgr, gray, COLOR_BGR2GRAY);

    // 2) Blur (노이즈 감소)
    GaussianBlur(gray, gray, Size(5, 5), 0);

    // 3) adaptive threshold (조명 변화에 강함)
    Mat bin;
    adaptiveThreshold(gray, bin, 255,
                      ADAPTIVE_THRESH_GAUSSIAN_C,
                      THRESH_BINARY,
                      31,  // blockSize(홀수)
                      7);  // C

    // 라인 색에 따라 반전
    if (LINE_MODE == BLACK_ON_WHITE) {
        bitwise_not(bin, bin); // 검은 라인을 흰색(255)로
    }

    // 4) morphology: 노이즈 제거 + 라인 연결
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(bin, bin, MORPH_OPEN, kernel, Point(-1, -1), 1);
    morphologyEx(bin, bin, MORPH_CLOSE, kernel, Point(-1, -1), 2);

    // 5) contour
    vector<vector<Point>> contours;
    findContours(bin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 디버그 기본
    Mat vis;
    cvtColor(bin, vis, COLOR_GRAY2BGR);

    if (contours.empty()) {
        debugRoiView = vis;
        return -1;
    }

    int bestIdx = -1;
    double bestArea = 0.0;
    for (int i = 0; i < (int)contours.size(); i++) {
        double a = contourArea(contours[i]);
        if (a > bestArea) { bestArea = a; bestIdx = i; }
    }

    if (bestIdx < 0 || bestArea < MIN_LINE_AREA) {
        debugRoiView = vis;
        return -1;
    }

    Moments m = moments(contours[bestIdx]);
    int cx = (m.m00 != 0.0) ? (int)(m.m10 / m.m00) : -1;

    // 디버그: 가장 큰 컨투어 + 중심
    drawContours(vis, contours, bestIdx, Scalar(0, 255, 0), 2);
    if (cx >= 0) {
        circle(vis, Point(cx, vis.rows / 2), 6, Scalar(0, 0, 255), -1);
    }

    debugRoiView = vis;
    return cx;
}

// =========================
// ArUco 인식 (OpenCV 버전 호환: Ptr<Dictionary>, Ptr<DetectorParameters> 사용)
// debugView: 마커 표시된 이미지
// =========================
static void detectAruco4x4(
    const Mat& frameBgr,
    vector<int>& ids,
    vector<vector<Point2f>>& corners,
    Mat& debugView
) {
    // Gray
    Mat gray;
    cvtColor(frameBgr, gray, COLOR_BGR2GRAY);

    // 조명 보정: CLAHE (인식률↑)
    Ptr<CLAHE> clahe = createCLAHE(2.5, Size(8, 8));
    Mat eq;
    clahe->apply(gray, eq);

    // 약한 블러
    GaussianBlur(eq, eq, Size(3, 3), 0);

    // ✅ Dictionary를 Ptr로 감싸서 만들기 (당신 헤더 요구사항 충족)
    Ptr<aruco::Dictionary> dict =
        makePtr<aruco::Dictionary>(aruco::getPredefinedDictionary(ARUCO_DICT));

    // ✅ DetectorParameters도 Ptr로 (시그니처 요구)
    Ptr<aruco::DetectorParameters> params = makePtr<aruco::DetectorParameters>();

    // 파라미터 튜닝(필요시 조금씩 조절)
    params->adaptiveThreshWinSizeMin = 3;
    params->adaptiveThreshWinSizeMax = 23;
    params->adaptiveThreshWinSizeStep = 10;
    params->adaptiveThreshConstant = 7;

    params->minMarkerPerimeterRate = 0.03;
    params->maxMarkerPerimeterRate = 4.0;

    params->polygonalApproxAccuracyRate = 0.03;
    params->minCornerDistanceRate = 0.05;
    params->minDistanceToBorder = 3;

    params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    params->cornerRefinementWinSize = 5;
    params->cornerRefinementMaxIterations = 30;
    params->cornerRefinementMinAccuracy = 0.1;

    // detect
    // (aruco.hpp가 Ptr<Dictionary>, Ptr<DetectorParameters>를 요구하는 환경에 맞춤)
    aruco::detectMarkers(eq, dict, corners, ids, params);

    // debug draw
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

    cout << "Start. ESC to quit.\n";
    cout << "OpenCV: " << CV_VERSION << "\n";

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        resize(frame, frame, Size(FRAME_W, FRAME_H));

        // 1) ArUco
        vector<int> ids;
        vector<vector<Point2f>> corners;
        Mat arucoDebug;
        detectAruco4x4(frame, ids, corners, arucoDebug);

        // 2) Line
        Mat lineDebugRoi;
        int lineCx = detectLineCenterX(frame, lineDebugRoi);

        bool markerDetected = !ids.empty();
        int markerId = markerDetected ? ids[0] : -1;

        bool lineFound = (lineCx >= 0);

        // steer: -1 ~ +1
        double steer = 0.0;
        if (lineFound) {
            double centerX = frame.cols / 2.0;
            double error = (double)lineCx - centerX;
            double normError = error / centerX;
            steer = clampDouble(KP * normError, -1.0, 1.0);
        }

        // 3) 상태(예시)
        string state;
        if (markerDetected) {
            state = "ARUCO_DETECTED";
            // TODO: 여기서 정지/감속/회전 등 이벤트 처리
            // 예) steer = 0.0; speed = 0.0;
        } else if (lineFound) {
            state = "LINE_FOLLOW";
        } else {
            state = "LINE_LOST";
            // TODO: 라인 분실 시 탐색 로직
        }

        // 4) 시각화
        Mat vis = arucoDebug.clone();

        // ROI 박스
        int y0 = (int)(FRAME_H * ROI_Y_START_RATIO);
        rectangle(vis, Rect(0, y0, FRAME_W, FRAME_H - y0), Scalar(255, 0, 0), 2);

        // 라인 중심을 전체 좌표에 표시(ROI 기준 x -> 전체 x 동일, y만 ROI로 이동)
        if (lineFound) {
            circle(vis, Point(lineCx, y0 + (FRAME_H - y0) / 2), 6, Scalar(0, 0, 255), -1);
        }

        char buf[256];
        snprintf(buf, sizeof(buf),
                 "State=%s | steer=%.2f | lineCx=%d | marker=%d",
                 state.c_str(), steer, lineCx, markerId);

        putText(vis, buf, Point(10, 30),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);

        imshow("main", vis);
        imshow("line_roi_debug", lineDebugRoi);

        int key = waitKey(1);
        if (key == 27) break; // ESC
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
