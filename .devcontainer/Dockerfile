FROM althack/ros2:humble-full 
# Optional, if no GUI application inside the devcontainer are needed, switch to:
# FROM althack/ros2:humble-dev 

RUN curl https://www.postgresql.org/media/keys/ACCC4CF8.asc | gpg --dearmor | tee /etc/apt/trusted.gpg.d/apt.postgresql.org.gpg >/dev/null
RUN echo "deb http://apt.postgresql.org/pub/repos/apt $(lsb_release -cs)-pgdg-testing main 15" | tee /etc/apt/sources.list.d/pgdg-testing.list



ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   # PC requirements
   ros-${ROS_DISTRO}-sensor-msgs-py \
   # PostgreSQL and PostGIS related
   postgresql-14 postgresql-14-postgis-3 \
   # mcap for ROS
   ros-${ROS_DISTRO}-rosbag2-storage-mcap \
   # Robot localization
   ros-${ROS_DISTRO}-robot-localization \
   # rviz satellite
   ros-${ROS_DISTRO}-rviz-satellite \
   git-lfs \
   python3-autopep8 \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# Install Foxglove Studio and ROS Bridge
RUN wget https://github.com/foxglove/studio/releases/download/v1.66.0/foxglove-studio-1.66.0-linux-amd64.deb -O foxglove.deb && \
    apt-get update && \
    apt-get install -y ./foxglove.deb && \
    apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-foxglove-bridge && \
    rm foxglove.deb && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

RUN pip install sqlalchemy shapely psycopg2-binary pyproj scipy pytest-postgresql


RUN sed -i 's/port = 5432/port = 5444/g' /etc/postgresql/14/main/postgresql.conf
RUN sed -i 's/scram-sha-256/trust/g' /etc/postgresql/14/main/pg_hba.conf

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
