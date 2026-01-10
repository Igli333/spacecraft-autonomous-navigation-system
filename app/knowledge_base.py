import sqlite3
import numpy as np


class KnowledgeBase:
    def __init__(self, db_file="kb.db"):
        self.db_file = db_file
        self._setupDB()

    def _setupDB(self):
        self.conn = sqlite3.connect(self.db_file)
        self.cursor = self.conn.cursor()

        self.cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS history
            (
                time REAL,
                rho_x REAL,
                rho_y REAL,
                rho_z REAL,
                rhoDot_x REAL,
                rhoDot_y REAL,
                rhoDot_z REAL,
                dv_x REAL,
                dv_y REAL,
                dv_z REAL
            )
            """)

        self.conn.commit()

    def log(self, time, rho, rho_dot, delta_v):
        rho = np.array(rho, dtype=float)
        rho_dot = np.array(rho_dot, dtype=float)
        delta_v = np.array(delta_v, dtype=float)

        self.cursor.execute(
            'INSERT INTO history VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)',
            (
                time,
                rho[0], rho[1], rho[2],
                rho_dot[0], rho_dot[1], rho_dot[2],
                delta_v[0], delta_v[1], delta_v[2]
            )
        )

        self.conn.commit()

    def close(self):
        self.conn.close()
