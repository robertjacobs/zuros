import sys
import MySQLdb

import rospy
import config_database

## Database class
class SensorsInDatabase(object):
	## Constructor
	def __init__(self):
		# Get the parameters from the config file
		self._sensorTable = config_database.database['mysql_sensor_table_name']
		self._sql = Database()

	## Gets all sensors from the database
	def get_all_sensors(self):
		query = "SELECT * FROM %s" % self._sensorTable
		return self._sql.execute_and_return(query)
	
	def get_value_by_communication_id(self, communication_id):
		query = "SELECT value FROM %s WHERE communication_id = '%s';" % (self._sensorTable, communication_id)
		return self._sql.execute_and_return(query)

## Database class
class Database(object):
	## Constructor
	def __init__(self, hostname=None, username=None, password=None, database=None):
		self._host = hostname or config_database.database['mysql_server']
		self._pass = password or config_database.database['mysql_password']
		self._user = username or config_database.database['mysql_user']
		self._db_url = database or config_database.database['mysql_db']
		self._db = MySQLdb.connect(self._host, self._user, self._pass, self._db_url)

	## Executes query on database
	def execute(self, query):
		cursor = self._db.cursor()
		cursor.execute(query)
		self._db.commit()
    
    ## Executes query and returns results
	def execute_and_return(self, query):
		cursor = self._db.cursor(MySQLdb.cursors.DictCursor)
		cursor.execute(query)
		return cursor.fetchall()
