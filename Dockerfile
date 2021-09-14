FROM ubuntu:18.04
WORKDIR /usr/src/app
RUN apt-get update && apt-get install -y \
    ninja-build openocd gcc-arm-none-eabi build-essential \
    python3 python3-pip lsb-release vim cmake

# RUN rm -rf /var/lib/apt/lists/*
# COPY . .
# RUN pip install --no-cache-dir -r requirements.txt