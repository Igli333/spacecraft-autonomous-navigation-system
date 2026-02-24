FROM python:3.13.10-alpine

WORKDIR .

COPY app.py /app.py
COPY planner_service.py /planner_service.py

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

EXPOSE 5004

CMD ["python", "app.py"]