import os

import requests
from flask import Flask, request, jsonify

import analyzer_service

app = Flask(__name__)


@app.route('/analyze', methods=["POST"])
def analyze():
    data = request.get_json()

    if not data:
        return jsonify({"error": "No JSON body provided"}), 400

    analyzed_data = analyzer_service.analyze(data)

    try:
        response = requests.post(
            f"{os.getenv('PLANNER_URI')}/plan",
            json=analyzed_data,
            timeout=5
        )

        response.raise_for_status()

    except requests.RequestException as e:
        return jsonify({"error": str(e)}), 502

    return jsonify(response.json()), response.status_code


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5003)
