CREATE TABLE IF NOT EXISTS delivery_table (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    delivery_id TEXT NOT NULL UNIQUE,
    vehicle_id TEXT NOT NULL,
    destination TEXT NOT NULL,
    receiver TEXT NOT NULL,
    order_time DATETIME DEFAULT CURRENT_TIMESTAMP,
    start_time DATETIME,
    arrive_time DATETIME,
    complete_time DATETIME,
    status TEXT DEFAULT 'ordered'
);
CREATE TABLE IF NOT EXISTS password_table (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    vehicle_id TEXT NOT NULL,
    delivery_id TEXT NOT NULL,
    pin_code TEXT NOT NULL UNIQUE,
    pin_type TEXT NOT NULL,        -- 'offboard' 또는 'onboard'
    created_time DATETIME DEFAULT CURRENT_TIMESTAMP,
    expire_time DATETIME,
    used INTEGER DEFAULT 0,
    attempt_count INTEGER DEFAULT 0
);
CREATE TABLE IF NOT EXISTS event_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    vehicle_id TEXT,
    event_type TEXT NOT NULL,
    delivery_id TEXT,
    detail TEXT,
    severity TEXT DEFAULT 'info'
);
CREATE INDEX IF NOT EXISTS idx_delivery_vehicle ON delivery_table(vehicle_id);
CREATE INDEX IF NOT EXISTS idx_password_delivery ON password_table(delivery_id);
CREATE INDEX IF NOT EXISTS idx_event_timestamp ON event_log(timestamp);
