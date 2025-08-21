-- Insert sample AMRs
INSERT INTO amr (name, ip_address, model, serial, firmware_version, last_update_date)
VALUES ('AMR-Unit-01', '192.168.100.217', 'Model-X', 'AMR001', 'v1.2.3', '2025-08-01 10:15:00'),
       ('AMR-Unit-02', '192.168.100.141', 'Model-Y', 'AMR002', 'v1.2.4', '2025-08-01 11:00:00'),
       ('AMR-Unit-03', '192.168.100.255', 'Model-Z', 'AMR003', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-0', '192.168.100.255', 'VirtualDevice', '0', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-1', '192.168.100.255', 'VirtualDevice', '1', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-2', '192.168.100.255', 'VirtualDevice', '2', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-3', '192.168.100.255', 'VirtualDevice', '3', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-4', '192.168.100.255', 'VirtualDevice', '4', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-5', '192.168.100.255', 'VirtualDevice', '5', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-6', '192.168.100.255', 'VirtualDevice', '6', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-7', '192.168.100.255', 'VirtualDevice', '7', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-8', '192.168.100.255', 'VirtualDevice', '8', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-9', '192.168.100.255', 'VirtualDevice', '9', 'v1.3.0', '2025-08-02 09:30:00'),
       ('Virtual-Device-10', '192.168.100.255', 'VirtualDevice', '10', 'v1.3.0', '2025-08-02 09:30:00')
;

-- Insert sample AMR statuses (linked to amr.serial)
INSERT INTO amr_status (amr_serial, state, x, y, speed, _zone, angle, created_at)
VALUES ('AMR001', 'RUNNING', 12.34, 56.78, 50, 'A구역', 90.0, '2025-08-02 10:00:00'),
       ('AMR001', 'CHARGING', 15.00, 60.12, 0, 'A충전소', 100.5, '2025-08-02 10:05:00'),
       ('AMR002', 'CHECKING', 5.6, 10.1, 0, 'B구역', 0.0, '2025-08-02 09:45:00')
;

-- Insert sample notifications
INSERT INTO notification (serial, title, content, _case, area, image, is_read, read_at, create_at)
VALUES ('AMR001', '사람 쓰러짐 감지', 'A구역에 사람 쓰러짐 감지', 'FALL', 'A구역', NULL, false, NULL,
        '2025-08-02 10:10:00'),
       ('AMR002', '안전 장비 미착용', 'B구역에 안전 장비 미착용 감지', 'EQUIPMENT', 'B구역', null,
        true, '2025-08-02 09:50:00', '2025-08-02 09:49:00')
;

-- Insert sample users
INSERT INTO _user (username, password)
-- username : test
-- password : test
VALUES ('test', '$2a$12$xmaVfibdTHwVXwkSoL3tyu0o8aJRU2eIjcQ5LImg.YiUbj5FOPXiG')
;
