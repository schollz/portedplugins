FROM armv7/armhf-debian
ENV DEBIAN_FRONTEND noninteractive

#RUN apt-get update RUN apt-get install -y git c++ cmake

# Installing necessary packages
RUN echo "deb http://archive.debian.org/debian stretch contrib main non-free" > /etc/apt/sources.list && \
    echo "deb http://archive.debian.org/debian-security stretch/updates main" >> /etc/apt/sources.list && \
    echo "deb-src http://archive.debian.org/debian stretch main" >> /etc/apt/sources.list 
RUN apt-get update 
RUN echo "hi"
RUN apt-get install -y --force-yes git cmake g++ make
WORKDIR /tmp/
RUN git clone https://github.com/schollz/portedplugins
WORKDIR /tmp/portedplugins
RUN apt-get install -y --force-yes wget zip tar
RUN wget https://github.com/Kitware/CMake/releases/download/v3.26.3/cmake-3.26.3.tar.gz
RUN tar -xvzf cmake-3.26.3.tar.gz
WORKDIR /tmp/portedplugins/cmake-3.26.3
RUN apt-get install -y --force-yes libssl-dev
RUN ./bootstrap
RUN make
RUN make install
WORKDIR /tmp/portedplugins
RUN make
RUN apt-get install -y --force-yes curl
RUN echo "Hi"
RUN curl --upload-file PortedPlugins.zip https://share.schollz.com/
