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
		self._sensorTypesTable = config_database.database['mysql_sensor_types_table_name']
		self._sql = Database()

	## Gets all sensors from the database
	def get_all_sensors(self):
		query = "SELECT * FROM %s" % self._sensorTable
		return self._sql.execute_and_return(query)
        
	## Gets all sensor types from the database
	def get_all_sensor_types(self):
		query = "SELECT * FROM %s" % self._sensorTypesTable
		return self._sql.execute_and_return(query)
        
	## Updates sensor value in the database
	def update_sensor_value(self, sensor):
		return self._sql.update_sensor_value(sensor)

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

	## Updates sensor value in the database
	def update_sensor_value(self, sensor):
		query = "UPDATE Sensor SET value='%s', interpreted_value='%s', last_interpreted_value='%s', last_updated='%s' WHERE id='%s' " % (
																																sensor['value'],
																																sensor['interpreted_value'],
																																sensor['last_interpreted_value'],
																																sensor['last_updated'],
																																sensor['id'])
		try:
			self.execute(query)
			return True
		except Exception as e:
			print "[ERROR] - exception raised while updating sensor information in database: %s" % e
			return False
