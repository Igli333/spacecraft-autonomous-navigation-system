FROM python:3.13.10-alpine

WORKDIR .

COPY app.py /app.py
COPY analyzer_service.py /analyzer_service.py

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

EXPOSE 5003

CMD ["python", "app.py"]