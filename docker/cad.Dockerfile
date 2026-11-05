FROM python:3.11-slim
RUN pip install --no-cache-dir cadquery-ocp==8.0.1.1.0 numpy
RUN sed -i 's|http://deb.debian.org|https://deb.debian.org|g' /etc/apt/sources.list.d/debian.sources \
    && apt-get -o Acquire::Retries=3 update \
    && apt-get -o Acquire::Retries=3 install -y --no-install-recommends libgl1 libxrender1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
ENTRYPOINT ["python3"]
