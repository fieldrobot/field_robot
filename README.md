Welcome to the Field Robot project!

Please refer to the project documentation at [https://github.com/fieldrobot/field_robot/wiki](https://github.com/fieldrobot/field_robot/wiki) for all information. Guides etc. are available in German and, partly, in English.

All contributors should consider [this guide](https://github.com/fieldrobot/field_robot/wiki/Nutzung-des-Repositories-(Deutsch)) on how to properly use the repository.

## Docker
For the ROS part of the Project there is a docker image and compose file. Just go to the `/dev_ws` folder and build/run the container via `docker-compose build` and `docker-compose up -d`. It should be noted that you have to use bash inside of the Container. If you want to be able to see GUIs outside of the Contianer, just add a `.env` file in the `/dev_ws` folder, defining the `DISPLAY` variable for X-Server integration.