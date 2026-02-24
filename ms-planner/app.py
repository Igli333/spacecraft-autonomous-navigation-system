from flask import Flask, request, jsonify

import planner_service

app = Flask(__name__)


@app.route("/plan", methods=["POST"])
def plan():
    data = request.get_json()

    if not data:
        return jsonify({"error": "No JSON body provided"}), 400

    result = planner_service.plan(data)

    return jsonify(result), 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5004)
