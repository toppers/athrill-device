#FROM ubuntu:20.04 as athrill-device-v850-builder
FROM kanetugu2015/athrill-device-v850:v1.0.0 as athrill-device-rh850-builder

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /root/workspace

#hakoniwa-core
RUN git clone --recurse-submodules https://github.com/toppers/hakoniwa-core.git

#athrill-target-rh850
RUN git clone --recurse-submodules -b RC-v0.A.2 https://github.com/toppers/athrill-target-rh850f1x.git

#atk2-sc1
RUN git clone https://github.com/mikoto2000/toppers-atk2-sc1.git

#cfg
RUN mkdir -p /root/workspace/toppers-atk2-sc1/cfg/cfg \
    && curl -L -O https://github.com/mitsut/cfg/releases/download/1.9.7/cfg-1.9.7-x86_64-unknown-linux-gnu.tar.gz \
    && tar xf cfg-1.9.7-x86_64-unknown-linux-gnu.tar.gz -C /root/workspace/toppers-atk2-sc1/cfg/cfg \
    && rm cfg-1.9.7-x86_64-unknown-linux-gnu.tar.gz

#install athrill
WORKDIR /root/workspace/athrill-target-rh850f1x/build_linux 
RUN make skip_clock_bugfix=true fix_intr_cpu_rcv_clr=true fix_intr_pmr_bugfix=true 


#install hakoniwa command
WORKDIR /root/workspace/hakoniwa-core/impl/asset/client/build
RUN bash build.bash cpp

ENV PATH /root/workspace/athrill-target-rh850f1x/athrill/bin/linux/:${PATH}
ENV PATH /root/workspace/hakoniwa-core/impl/asset/client/build/cpp/:${PATH}
WORKDIR /root/workspace
RUN ln -s /root/workspace/athrill-target-rh850f1x/athrill /root/workspace/athrill
WORKDIR /root/workspace/athrill-device
