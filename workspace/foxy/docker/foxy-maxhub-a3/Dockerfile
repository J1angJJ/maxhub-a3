ARG BASE_IMAGE=ubuntu-env:foxy-user

FROM ${BASE_IMAGE}

ENV MAXHUB_A3_PROJECT=foxy-maxhub-a3 \
    CMAKE_PREFIX_PATH=/opt/maxhub-a3/arm_control_sdk \
    LD_LIBRARY_PATH=/opt/maxhub-a3/arm_control_sdk/lib:/opt/maxhub-a3/arm_control_sdk/poco/lib

WORKDIR /workspace
