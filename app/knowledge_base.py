import sqlite3
import numpy as np


class KnowledgeBase:
    def __init__(self, db="kb.db"):
        self.conn = sqlite3.connect(db, check_same_thread=False)
        self.cursor = self.conn.cursor()
        self._init()

    def _init(self):
        self.cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS history_force
            (
                time         REAL,
                -- Hill frame
                rho_x        REAL,
                rho_y        REAL,
                rho_z        REAL,
                rhoDot_x     REAL,
                rhoDot_y     REAL,
                rhoDot_z     REAL,
                -- Action
                force_x      REAL,
                force_y      REAL,
                force_z      REAL,

                phase        VARCHAR,
                risk         BOOLEAN,

                success      BOOLEAN,
                abort        BOOLEAN,
                abort_reason VARCHAR
            );
            """)

        self.cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS history_torque
            (
                time         REAL,
                dist         REAL,

                sigma_x      REAL,
                sigma_y      REAL,
                sigma_z      REAL,

                omega_x      REAL,
                omega_y      REAL,
                omega_z      REAL,

                torque_x     REAL,
                torque_y     REAL,
                torque_z     REAL,

                phase        VARCHAR,

                success      BOOLEAN,
                abort        BOOLEAN,
                abort_reason VARCHAR
            );
            """)

        self.conn.commit()

    def log_force(self, **kwargs):
        self.cursor.execute(
            """
            INSERT INTO history_force
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);
            """, tuple(kwargs.values())
        )

        self.conn.commit()

    def log_torque(self, **kwargs):
        self.cursor.execute(
            """
            INSERT INTO history_torque
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);
            """, tuple(kwargs.values())
        )

        self.conn.commit()

    def learnedSafeSpeed(self, range_, window=1.0):
        rows = self.conn.execute(
            """
            SELECT SQRT(rhoDot_x * rhoDot_x + rhoDot_y * rhoDot_y + rhoDot_z * rhoDot_z) AS speed
            FROM history_force
            WHERE phase IN ('MID', 'CLOSE', 'DOCKING')
              AND ABS(
                          SQRT(rho_x * rho_x + rho_y * rho_y + rho_z * rho_z) - ?
                  ) < ?
              AND abort = 0
              AND (success IS NULL OR success = 1)
            ORDER BY speed ASC
            """, (range_, window)
        ).fetchall()

        if len(rows) < 5:
            return None

        speeds = np.array([r[0] for r in rows])
        return np.quantile(speeds, 0.2)

    def learnedMaxForce(self, range_, window=1.0):
        rows = self.conn.execute(
            """
            SELECT SQRT(force_x * force_x + force_y * force_y + force_z * force_z) AS force
            FROM history_force
            WHERE phase IN ('MID', 'CLOSE', 'DOCKING')
              AND ABS(
                          SQRT(rho_x * rho_x + rho_y * rho_y + rho_z * rho_z) - ?
                  ) < ?
              AND abort = 0
            """, (range_, window)
        ).fetchall()

        if len(rows) < 5:
            return None

        forces = np.array([r[0] for r in rows])
        return np.quantile(forces, 0.8)

    def captureSuccessEnvelope(self, dist, window=0.05):
        rows = self.conn.execute(
            """
            SELECT ABS(rhoDot_x) AS v_parallel
            FROM history_force
            WHERE phase = 'DOCKING'
              AND ABS(ABS(rho_x) - ?) < ?
              AND success = 1
            ORDER BY v_parallel ASC
            """, (dist, window)
        ).fetchall()

        if len(rows) < 3:
            return None

        v = np.array([r[0] for r in rows])
        return np.quantile(v, 0.9)

    def learnedMaxTorque(self, dist):
        rows = self.conn.execute(
            """
            SELECT AVG(ABS(torque_x) + ABS(torque_y) + ABS(torque_z)) / 3.0
            FROM history_torque
            WHERE dist < ?
              AND success = 1
            """, (dist,)
        ).fetchall()

        if rows[0][0] is None:
            return None

        return float(rows[0][0])

    def captureAttitudeEnvelope(self, dist):
        rows = self.conn.execute(
            """
            SELECT MAX(
                           SQRT(sigma_x * sigma_x + sigma_y * sigma_y + sigma_z * sigma_z)
                   )
            FROM history_torque
            WHERE dist < ?
              AND success = 1
            """, (dist,)
        ).fetchall()

        if rows[0][0] is None:
            return None

        return float(rows[0][0])

    def abortProbabilityTorque(self, dist, sigma_norm, omega_norm):
        rows = self.conn.execute(
            """
            SELECT COUNT(*), SUM(abort)
            FROM history_torque
            WHERE dist < ?
              AND SQRT(
                          sigma_x * sigma_x + sigma_y * sigma_y + sigma_z * sigma_z
                  ) > ?
              AND SQRT(
                          omega_x * omega_x + omega_y * omega_y + omega_z * omega_z
                  ) > ?
            """, (dist, sigma_norm, omega_norm)
        ).fetchall()

        total, aborts = rows[0]
        if total == 0:
            return 0.0

        return aborts / total
