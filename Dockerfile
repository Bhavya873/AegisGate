FROM python:3.12-alpine
COPY demo /site
CMD python3 -m http.server $PORT --directory /site
