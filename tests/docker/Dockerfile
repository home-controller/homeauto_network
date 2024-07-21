# Fetch ubuntu image
FROM ubuntu:22.04

# Install Python on image
RUN \
    apt-get update && \
    apt-get install -y python3 && \
    apt-get install -y build-essential
    
# Create a directory for our tests
RUN mkdir /tests

# Copy in our Python script
COPY test.py /tests/test.py

# Copy in our program under test
COPY main.c /tests/main.c

# Command that will be invoked when container starts
ENTRYPOINT ["python3", "tests/test.py"]