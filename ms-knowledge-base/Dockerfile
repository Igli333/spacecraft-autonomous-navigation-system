FROM python:3.13.10-alpine

WORKDIR .

COPY app.py /app.py
COPY database.py /database.py
COPY knoweldge_base_service.py /knoweldge_base_service.py

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

EXPOSE 5001

CMD ["python", "app.py"]