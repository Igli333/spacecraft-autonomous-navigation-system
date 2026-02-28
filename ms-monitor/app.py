import json
import os

import requests

from flask import Flask, request, jsonify
from monitoring_service import monitor

app = Flask(__name__)


@app.route('/monitor', methods=['POST'])
def monitoring():
    data = request.get_json()

    if not data:
        return jsonify({"error": "No JSON body provided"}), 400

    monitored_data = monitor(data['data'])

    try:
        response = requests.post(
            f"{os.getenv('ANALYZER_URI')}/analyze",
            json={
                'monitored_data': monitored_data,
                'config': data['config']
            },
            timeout=5
        )

        response.raise_for_status()
        command = response.json()
    except requests.RequestException as e:
        return jsonify({"error": str(e)}), 502

    return jsonify(command), response.status_code


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5002)
