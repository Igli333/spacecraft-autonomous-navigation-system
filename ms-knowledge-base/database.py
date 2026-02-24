import os
import psycopg2


class Database:
    def __init__(self):
        self.conn = psycopg2.connect(
            dbname=os.getenv("DATABASE_NAME"),
            user=os.getenv("DATABASE_USER"),
            password=os.getenv("DATABASE_PASSWORD"),
            host=os.getenv("DATABASE_HOST"),
            port=os.getenv("DATABASE_PORT")
        )
        self.cursor = self.conn.cursor()
        self.init_tables()

    def init_tables(self):
        self.cursor.execute(
            """
                CREATE TABLE IF NOT EXISTS history_force (
                    time DOUBLE PRECISION,
    
                    -- Hill frame
                    rho_x  DOUBLE PRECISION,
                    rho_y DOUBLE PRECISION,
                    rho_z DOUBLE PRECISION,
                    rhoDot_x DOUBLE PRECISION,
                    rhoDot_y DOUBLE PRECISION,
                    rhoDot_z DOUBLE PRECISION,
    
                    -- Action
                    force_x DOUBLE PRECISION,
                    force_y DOUBLE PRECISION,
                    force_z DOUBLE PRECISION,
    
                    phase VARCHAR,
                    risk BOOLEAN,
    
                    success BOOLEAN,
                    abort BOOLEAN,
                    abort_reason VARCHAR
                );
            """
        )

        self.cursor.execute(
            """
                CREATE TABLE IF NOT EXISTS history_torque (
                    time DOUBLE PRECISION,
                    dist DOUBLE  PRECISION,
    
                    sigma_x DOUBLE PRECISION,
                    sigma_y DOUBLE PRECISION,
                    sigma_z DOUBLE PRECISION,
    
                    omega_x DOUBLE PRECISION,
                    omega_y DOUBLE PRECISION,
                    omega_z DOUBLE PRECISION,
    
                    torque_x DOUBLE PRECISION,
                    torque_y DOUBLE PRECISION,
                    torque_z DOUBLE PRECISION,
    
                    phase VARCHAR,
    
                    success BOOLEAN,
                    abort BOOLEAN,
                    abort_reason VARCHAR
                );
            """
        )

        self.conn.commit()

    def close(self):
        self.cursor.close()
        self.conn.close()
