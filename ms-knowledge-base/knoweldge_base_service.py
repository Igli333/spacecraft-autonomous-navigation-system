from database import Database
import numpy as np

database = Database()


def log_force(data):
    database.cursor.execute(
        """
        INSERT INTO history_force
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s);
        """, tuple(data.values())
    )

    database.conn.commit()


def log_torque(data):
    database.cursor.execute(
        """
        INSERT INTO history_torque
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s);
        """, tuple(data.values())
    )

    database.conn.commit()


def learnedSafeSpeed(range_, window=1.0):
    database.cursor.execute(
        """
        SELECT SQRT(rhoDot_x * rhoDot_x + rhoDot_y * rhoDot_y + rhoDot_z * rhoDot_z) AS speed
        FROM history_force
        WHERE phase IN ('MID', 'CLOSE', 'DOCKING')
          AND ABS(
                      SQRT(rho_x * rho_x + rho_y * rho_y + rho_z * rho_z) - %s
              ) < %s
          AND abort = FALSE
          AND (success IS NULL OR success = TRUE)
        ORDER BY speed ASC
        """, (range_, window)
    )

    rows = database.cursor.fetchall()

    if len(rows) < 5:
        return None

    speeds = np.array([r[0] for r in rows])
    return np.quantile(speeds, 0.2)


def learnedMaxForce(range_, window=1.0):
    database.cursor.execute(
        """
        SELECT SQRT(force_x * force_x + force_y * force_y + force_z * force_z) AS force
        FROM history_force
        WHERE phase IN ('MID', 'CLOSE', 'DOCKING')
          AND ABS(
                      SQRT(rho_x * rho_x + rho_y * rho_y + rho_z * rho_z) - %s
              ) < %s
          AND abort = FALSE
        """, (range_, window)
    )

    rows = database.cursor.fetchall()

    if len(rows) < 5:
        return None

    forces = np.array([r[0] for r in rows])
    return np.quantile(forces, 0.8)


def captureSuccessEnvelope(dist, window=0.05):
    database.cursor.execute(
        """
        SELECT ABS(rhoDot_x) AS v_parallel
        FROM history_force
        WHERE phase = 'DOCKING'
          AND ABS(ABS(rho_x) - %s) < %s
          AND success = TRUE
        ORDER BY v_parallel ASC
        """, (dist, window)
    )

    rows = database.cursor.fetchall()

    if len(rows) < 3:
        return None

    v = np.array([r[0] for r in rows])
    return np.quantile(v, 0.9)


def learnedMaxTorque(dist):
    database.cursor.execute(
        """
        SELECT AVG(ABS(torque_x) + ABS(torque_y) + ABS(torque_z)) / 3.0
        FROM history_torque
        WHERE dist < %s
          AND success = TRUE
        """, (dist,)
    )

    rows = database.cursor.fetchall()

    if rows[0][0] is None:
        return None

    return float(rows[0][0])


def captureAttitudeEnvelope(dist):
    database.cursor.execute(
        """
        SELECT MAX(
                       SQRT(sigma_x * sigma_x + sigma_y * sigma_y + sigma_z * sigma_z)
               )
        FROM history_torque
        WHERE dist < %s
          AND success = TRUE
        """, (dist,)
    )

    rows = database.cursor.fetchall()

    if rows[0][0] is None:
        return None

    return float(rows[0][0])


def abortProbabilityTorque(dist, sigma_norm, omega_norm):
    database.cursor.execute(
        """
        SELECT COUNT(*), SUM(abort::int)
        FROM history_torque
        WHERE dist < %s
          AND SQRT(
                      sigma_x * sigma_x + sigma_y * sigma_y + sigma_z * sigma_z
              ) > %s
          AND SQRT(
                      omega_x * omega_x + omega_y * omega_y + omega_z * omega_z
              ) > %s
        """, (dist, sigma_norm, omega_norm)
    )

    rows = database.cursor.fetchall()

    total, aborts = rows[0]
    if total == 0:
        return 0.0

    return aborts / total
