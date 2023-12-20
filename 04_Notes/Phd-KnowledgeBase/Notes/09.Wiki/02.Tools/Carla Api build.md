## Carla Prerequesites

```sh
sudo apt-get install qemu binfmt-support qemu-user-static
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

### dockerfile build env

```Dockerfile
FROM ubuntu:18.04

USER root

#ARG EPIC_USER=user
#ARG EPIC_PASS=pass
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update ; \
  apt-get install -y wget software-properties-common && \
  add-apt-repository ppa:ubuntu-toolchain-r/test && \
  wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|apt-key add - && \
  apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-8 main" && \
  apt-get update ; \
  apt-get install -y build-essential \
    clang-8 \
    lld-8 \
    g++-7 \
    cmake \
    ninja-build \
    libvulkan1 \
    python \
    python-pip \
    python-dev \
    python3-dev \
    python3-pip \
    libpng-dev \
    libtiff5-dev \
    libjpeg-dev \
    tzdata \
    sed \
    curl \
    unzip \
    autoconf \
    libtool \
    rsync \
    libxml2-dev \
    git \
    aria2 && \
  pip3 install -Iv setuptools==47.3.1 && \
  pip3 install distro && \
  update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 && \
  update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180

RUN useradd -m carla
COPY --chown=carla:carla . /home/carla
USER carla
WORKDIR /home/carla
ENV UE4_ROOT /home/carla/UE4.26

#RUN git clone --depth 1 -b carla "https://${EPIC_USER}:${EPIC_PASS}@github.com/CarlaUnreal/UnrealEngine.git" ${UE4_ROOT}

#RUN cd $UE4_ROOT && \
#  ./Setup.sh && \
#  ./GenerateProjectFiles.sh && \
#  make

WORKDIR /home/carla/

```

### Carla build PythonAPi

```Dockerfile
FROM carla-prerequisites:latest

ARG GIT_BRANCH

USER carla
WORKDIR /home/carla

RUN cd /home/carla/ && \
  if [ -z ${GIT_BRANCH+x} ]; then git clone --depth 1 https://github.com/carla-simulator/carla.git; \
  else git clone --depth 1 --branch $GIT_BRANCH https://github.com/carla-simulator/carla.git; fi && \
  cd /home/carla/carla && \
#  ./Update.sh && \
#  make CarlaUE4Editor && \
  make PythonAPI && \
#  make build.utils && \
#  make package && \
  rm -r /home/carla/carla/Dist

WORKDIR /home/carla/carla


```


#### Build command
```sh
docker build . --build-arg="GIT_BRANCH=0.9.13" --platform linux/arm64 -f ./Prerequisites.Dockerfile -t carla-prerequisites:latest
docker build . --build-arg="GIT_BRANCH=0.9.13" --platform linux/arm64 -f ./Carla.Dockerfile -t carla-api:latest
```


#### On nvidia agx

sudo apt-get install libgeos-dev
sudo python3 -m easy_install package.egg

### On p1 server

DISPLAY= ./CarlaUE4.sh -opengl -carla-server -benchmark -fps=10