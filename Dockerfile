FROM python:3.8

RUN pip install pipenv==2018.11.26

ENV PROJECT_DIR /usr/local/src/automation

WORKDIR ${PROJECT_DIR}

COPY ./Pipfile ./Pipfile.lock ./scripts/automation.py ./

RUN pipenv install --system --deploy

ENTRYPOINT [ "python", "./automation.py" ]
