ARG BASE_IMAGE=ubuntu-env:foxy-user

FROM ${BASE_IMAGE}

ARG USERNAME=developer

USER root

COPY requirements.txt /tmp/foxy-maxhub-a3-requirements.txt

RUN python3 -m pip install --no-cache-dir \
      "importlib-metadata>=4.13,<8" \
      -r /tmp/foxy-maxhub-a3-requirements.txt

USER ${USERNAME}

ENV MAXHUB_A3_PROJECT=foxy-maxhub-a3 \
    CMAKE_PREFIX_PATH=/opt/maxhub-a3/arm_control_sdk \
    LD_LIBRARY_PATH=/opt/maxhub-a3/arm_control_sdk/lib:/opt/maxhub-a3/arm_control_sdk/poco/lib

WORKDIR /workspace/rl_ws
