#!/bin/bash
# set +x

HOST="localhost"
PORT=5444
USER="postgres"
DATABASE="kieswerk"

CONN_ARGS="-h $HOST -p $PORT -U $USER"

export PGPASSWORD="postgres"

BASE_DIR=$(dirname -- "$0")

sudo service postgresql start

psql $CONN_ARGS -c "CREATE DATABASE $DATABASE;"
psql $CONN_ARGS -d $DATABASE -f $BASE_DIR/building.sql
psql $CONN_ARGS -d $DATABASE -f $BASE_DIR/landuse.sql
psql $CONN_ARGS -d $DATABASE -f $BASE_DIR/power_lines.sql
psql $CONN_ARGS -d $DATABASE -f $BASE_DIR/power_pois.sql
psql $CONN_ARGS -d $DATABASE -f $BASE_DIR/route_multiline.sql
psql $CONN_ARGS -d $DATABASE -f $BASE_DIR/route_polygon.sql
psql $CONN_ARGS -d $DATABASE -f $BASE_DIR/setup_database.sh
psql $CONN_ARGS -d $DATABASE -f $BASE_DIR/wikipedia_linestring.sql
psql $CONN_ARGS -d $DATABASE -f $BASE_DIR/wikipedia_multiline.sql
psql $CONN_ARGS -d $DATABASE -f $BASE_DIR/wikipedia_polygon.sql