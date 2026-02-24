from flask import Flask, jsonify, request
import knoweldge_base_service
from database import Database

app = Flask(__name__)
database = Database()


@app.route("/log/force", methods=["POST"])
def log_force():
    try:
        knoweldge_base_service.log_force(**request.json)
        return jsonify({"status": "ok"}), 201
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/log/torque", methods=["POST"])
def log_torque():
    try:
        knoweldge_base_service.log_torque(**request.json)
        return jsonify({"status": "ok"}), 201
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/analytics/safe-speed")
def learned_safe_speed():
    range_ = float(request.args.get("range"))
    window = float(request.args.get("window", 1.0))

    result = knoweldge_base_service.learnedSafeSpeed(range_, window)
    return jsonify({"value": result})


@app.route("/analytics/max-force")
def learned_max_force():
    range_ = float(request.args.get("range"))
    window = float(request.args.get("window", 1.0))

    result = knoweldge_base_service.learnedMaxForce(range_, window)
    return jsonify({"value": result})


@app.route("/analytics/success-envelope")
def capture_success_envelope():
    dist = float(request.args.get("dist"))
    window = float(request.args.get("window", 0.05))

    result = knoweldge_base_service.captureSuccessEnvelope(dist, window)
    return jsonify({"value": result})


@app.route("/analytics/max-torque")
def learned_max_torque():
    dist = float(request.args.get("dist"))

    result = knoweldge_base_service.learnedMaxTorque(dist)
    return jsonify({"value": result})


@app.route("/analytics/attitude-envelope")
def capture_attitude_envelope():
    dist = float(request.args.get("dist"))

    result = knoweldge_base_service.captureAttitudeEnvelope(dist)
    return jsonify({"value": result})


@app.route("/analytics/abort-probability-torque")
def abort_probability_torque():
    dist = float(request.args.get("dist"))
    sigma_norm = float(request.args.get("sigma_norm"))
    omega_norm = float(request.args.get("omega_norm"))

    result = knoweldge_base_service.abortProbabilityTorque(dist, sigma_norm, omega_norm)
    return jsonify({"value": result})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
