-- Table: amr
CREATE TABLE IF NOT EXISTS amr
(
    id               BIGSERIAL PRIMARY KEY,
    name             VARCHAR(255) NOT NULL,
    ip_address       VARCHAR(255) NOT NULL,
    model            VARCHAR(255) NOT NULL,
    serial           VARCHAR(255) NOT NULL UNIQUE,
    firmware_version VARCHAR(255) NOT NULL,
    last_update_date TIMESTAMP    NOT NULL
);

-- Table: amr_status (foreign key to amr.serial)
CREATE TABLE IF NOT EXISTS amr_status
(
    id            BIGSERIAL PRIMARY KEY,
    amr_serial    VARCHAR(255)                        NOT NULL,
    status        VARCHAR(255)                        NOT NULL,
    battery_level INTEGER                             NOT NULL,
    x             DOUBLE PRECISION                    NOT NULL,
    y             DOUBLE PRECISION                    NOT NULL,
    speed         DOUBLE PRECISION                    NOT NULL,
    created_at    TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL,
    FOREIGN KEY (amr_serial) REFERENCES amr (serial) ON DELETE CASCADE
);

-- Index for latest status by AMR
CREATE INDEX idx_amr_status_amr_serial_created_at
    ON amr_status (amr_serial, created_at DESC);


CREATE TABLE IF NOT EXISTS fcm_token
(
    id    BIGSERIAL PRIMARY KEY,
    token VARCHAR(255) NOT NULL
);