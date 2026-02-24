FROM python:3.13.10-alpine

WORKDIR .

COPY app.py /app.py
COPY utils.py /utils.py
COPY monitoring_service.py /monitoring_service.py

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

EXPOSE 5002

CMD ["python", "app.py"]