ARG BASE_IMAGE=ubuntu-env:noetic-user

FROM ${BASE_IMAGE}

ARG NOETIC_USER=developer

COPY requirements.txt /tmp/noetic-maxhub-a3-requirements.txt

RUN python3 -m pip install --user --no-cache-dir \
      -r /tmp/noetic-maxhub-a3-requirements.txt \
    && rm -f /tmp/noetic-maxhub-a3-requirements.txt

ENV PATH=/home/${NOETIC_USER}/.local/bin:${PATH}
