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
            CREATE TABLE IF NOT EXISTS history
            (
                run_id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
                time REAL,
                -- Hill frame
                rho_x REAL,
                rho_y REAL,
                rho_z REAL,
                rhoDot_x REAL,
                rhoDot_y REAL,
                rhoDot_z REAL,
                -- Action
                force_x REAL,
                force_y REAL,
                force_z REAL,
                
                phase VARCHAR,
                risk REAL,
                
                success INTEGER,
                abort INTEGER,
                abort_reasoon VARCHAR
            );
            """)

        self.conn.commit()

    def log(self, **kwargs):
        self.cursor.execute("""
            INSERT INTO history VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?);
            """, tuple(kwargs.values())
        )

        print(kwargs.items())

        self.conn.commit()

    def learnedSafeSpeed(self, range_):
       self.cursor.execute("""
            SELECT SQRT(rhoDot_x*rhoDot_x + rhoDot_y*rhoDot_y + rhoDot_z*rhoDot_z)
            FROM history
                WHERE abort = 0
       """)

       speeds = [r[0] for r in self.cursor.fetchall()]
       return np.percentile(speeds, 70) if speeds else None