#!usr/bin/env python
# Date: Nov 2015
# Developer: Chao Liu
# Description: Script to install PostgreSQL on Windows 7 and Linux(Ubuntu 14.04)
# 	Windows, Use the administrator role to execute the process. After installing it, there is a superuser, 
#	the name is the same as your system account. And going into the direction that the file is installed, 
#	and under 'bin' direction, you will find the client file 'pgAdmin 3'. Double Click it. Then create a 
#	server without password at the graphical interface, the username is your system account. Notes, 
#	when you open the server created by you again, you will set password for the account.
#	Linux, after installation, there are a superuser and a database. The name of the superuser is the name
#	of your system account, the password is the same as your system password. The database is named as test.     
#
# Installation Operations:
#	* Windows 7: 
#		1. Run 'python pgsql.py setup' in DOS going into the current direction of the file pgsql.py
#		2. Input the version being installed (e.g: 9.4.5)
#		3. Click 'Enter'  
#	* Linux (Ubuntu 14.04):
#		1. Run 'python pgsql.py setup' in Terminal going into the current direction of the file pgsql.py
#		2. Input your system account password
#		3. Click 'Enter'


import os
import sys
import getpass
import platform
import urllib2
import zipfile


#*******Windows (definition of some variable)**************  
# ask for user to input version being installed. the default version is 9.3.10  
version = '9.3.10'
#define a url to get the download path.The default system bite is 32 bit
url = r'http://get.enterprisedb.com/postgresql/postgresql-'+version+'-2-windows-binaries.zip'
# define a variable to reserve the zip file 
zipdir = r'D:/PostgreSQL'+version+'.zip'
# define a variable to describe the installation direction 
unziptodir = r'D:/PostgreSQL_test'+version+'/'


#**********Linux (define some variable)***************************************************
#define a variable to store password, the default password is '123456'
password = '123456'
#define a variable to create a database
dbname = 'test' 


#******Windows, Functions*****************

# Judge the type of system
def iswin32():
    return ('32bit','WindowsPE') == platform.architecture()

# according to the type of system, make the downloading url
def getUrl():
	if iswin32():
		url = r'http://get.enterprisedb.com/postgresql/postgresql-'+version+'-2-windows-binaries.zip'
	else:
		url = r'http://get.enterprisedb.com/postgresql/postgresql-'+version+'-2-windows-x64-binaries.zip'

def getSavePath():
	zipdir = r'D:/PostgreSQL'+version+'.zip' 
	unziptodir = r'D:/PostgreSQL'+version+'/'

#------------------------------------------------------------
def version_pg():
	version = raw_input('Please input being installed version:')
	if version == '':
		version = '9.3.10'
	getUrl()
	getSavePath()
#------------------------------------------------------------

def judgeInit(command):
	if os.system(command) == 0:
		return 'Succussfully'
	else:
		print 'Operating command: "'+command +'" Failed!!!'
		return 'Failed',exit()

def init_w():
	path = unziptodir+'pgsql/'
	print 'Start Initdb......'
	print 'Initdb '+judgeInit(path+'bin/initdb.exe -D '+path+'data -E UTF8 --locale=C')
	print 'Regester Windows Service '+judgeInit(path+'bin/pg_ctl.exe register -D '+path+'data -N pgsql')
	print 'Start database '+judgeInit('net start pgsql')
	print 'Finishing initdb'

def judgeConfig(name,dir,env):
	_winreg.SetValueEx(env,name,0,_winreg.REG_SZ,dir)
	item,type =  _winreg.QueryValueEx(env,name)
	if dir in item:
		return 'Succussfully'
	else:
		print 'set '+name +'Failed!!!!!'
		return 'Failed',exit()

def config():
	if not os.path.exists(unziptodir):
		print 'Sorry, the directory is not defined!'
		exit()
	env = _winreg.OpenKeyEx(_winreg.HKEY_LOCAL_MACHINE,r"SYSTEM\ControlSet001\Control\Session Manager\Environment",0,_winreg.KEY_SET_VALUE|_winreg.KEY_READ)
	print 'Start configuration....'
	print 'set environment variable pghome '+judgeConfig('PGHOME',unziptodir+'pgsql',env)
	print 'set environment variable pgdata '+judgeConfig('PGDATA','%PGHOME%/data',env)
	print 'set environment variable pglib '+judgeConfig('PGLIB','%PGHOME%/lib',env)
	print 'set environment variable pghost '+judgeConfig('PGHOST','localhost',env)
	print 'add path to environment variable path '+judgeConfig('PATH','%PGHOME%/bin;'+os.environ['Path'],env)
	_winreg.CloseKey(env)
	print 'Finishing Configuration'

def uzfileFunc():
	if not os.path.exists(zipdir):
		print 'No such File or Directory'
		exit()
	uzfile = zipfile.ZipFile(zipdir)
	for name in uzfile.namelist():
		name = name.replace('\\','/')
		if name.endswith('/') : 
			newdir = os.path.join(unziptodir.replace('\\','/'),name)
			print 'newdir:'+newdir
			if not os.path.exists(newdir):
				os.makedirs(newdir)
		else:
			ext_filename = os.path.join(unziptodir,name)
			ext_dir = os.path.dirname(ext_filename)
			print 'ext_dir:'+ext_dir
			if not os.path.exists(ext_dir):
				os.mkdir(ext_dir,0777)
			outfile = open(ext_filename,'wb')
			outfile.write(uzfile.read(name))
			outfile.close()
	config()

def dlFile():
	zfile = urllib2.urlopen(url)
	zipcontent = zfile.read()
	with open(zipdir,'wb') as zd:
		zd.write(zipcontent)
	zfile.close()
	uzfileFunc()

def unpsql_w():
	if os.system('net stop pgsql') == 0:
		print 'stop service pgsql'
		if os.system('sc delete pgsql'):
			print 'pgsql service is deleted successfully'
		else:
			print 'delete Failed'
	else:
		print 'close service pgsql failed'



#************Linux,Functions****************
#------------------------------------------------------
def getPwd():
	password = raw_input('Please input your system password:')
	if password == '':
		password = '123456'
#------------------------------------------------------



#defien a function install_sta to construct 
# the statement of installing PostgreSQL
def install_sta():
	return "sudo apt-get -y install postgresql postgresql-contrib"

def operate(statement):
	if os.system('echo "'+statement+'" | sudo -i -u postgres psql') == 0:
		return 1
	else:
		return 0

def init_l():
	#define a variable to obtain the username
	suname = getpass.getuser()
	#define a variable to store a statement of creating user
	state_usr = "create user "+suname+" WITH superuser createdb createrole password '"+password+"'"  
	#define a variable to store a stateent of creating dbsudo
	state_db = 'create database '+dbname+' WITH owner '+suname
	if operate(state_usr) == 1:
		print 'create user '+suname+' successfully'
		if operate(state_db) == 1:    
			print 'create database '+dbname+' successfully'
		else:
			print 'creating databalse FAILD'
	else:
		print 'creating user FAILD' 

def install():
	#define a variable 'update' to update the repository of PostgreSQL 
	# and executing the statement with on password
	update = 'echo '+password+' | sudo -S apt-get update'
	# Calling the function os.system() to execute system command 
	if os.system(update) == 0:
		print '-------- update successful --------'
		if os.system(install_sta()) == 0:
			print '------- install successful -------'
		else :
			print '??????? install FAILD ????????'
	else :
		print '??????? update FAILD ???????'

#defien a function uninstall_sta to construct the statement of deleting packages related to postgres
# typing the command 'dpkg -l | grep postgres', 
# You could check these packages 
def uninstall_sta():
	return 'echo '+password+' | sudo -S apt-get -y --purge remove postgresql*'

#define a function to construct the statement of deleting some configuration folders
# /var/lib/postgresql/,
# /var/log/postgresql/,
# /etc/postgresql/
def unins_folders_sta():
	return 'sudo rm -rf /var/lib/postgresql/ && sudo rm -rf /var/log/postgresql/ && sudo rm -rf /etc/postgresql/'

def uninstall():
	print '---------- uninstall ----------------'
	if (os.system(uninstall_sta()) == 0 & os.system(unins_folders_sta()) == 0):
		print '------- uninstall successfully --------'
	else:
		print '??????? Uninstall FAILD ????????'



#----------***********************------------------
def psql_w():
	if sys.argv[1] == 'setup':
		version_pg()
		dlFile()
		#uzfileFunc()
		#config()
		init_w()
	elif sys.argv[1] == 'uninstall':
		unpsql_w()
	elif sys.argv[1] == 'test':
		print 'test'

def psql_l():
	if sys.argv[1] == 'setup':
		getPwd()
		install()
		init_l()
	elif sys.argv[1] == 'uninstall':
		uninstall()
	elif sys.argv[1] == 'test':
		print 'test'

#========================
# MAIN FUNCTION
if __name__ == '__main__':
	print 'Start...'
	#Automatically recognize Operating system
	if 'Windows' in platform.system():
		import _winreg
		psql_w()
	if 'Linux' in platform.system():
		psql_l()
	print 'Succussfully'





#References:
#http://www.postgresonline.com/journal/archives/172-Starting-PostgreSQL-in-windows-without-install.html
#http://www.petrikainulainen.net/programming/tips-and-tricks/installing-postgresql-9-1-to-windows-7-from-the-binary-zip-distribution/
#https://docs.python.org/2/library/_winreg.html
#http://www.oschina.net/code/snippet_89296_9122
#https://www.digitalocean.com/community/tutorials/how-to-install-and-use-postgresql-on-ubuntu-14-04
#http://askubuntu.com/questions/32730/how-to-remove-postgres-from-my-installation
#https://www.digitalocean.com/community/tutorials/how-to-install-and-use-postgresql-on-ubuntu-14-04
#http://askubuntu.com/questions/470383/how-to-avoid-prompt-password-for-sudo
#http://superuser.com/questions/164553/automatically-answer-yes-when-using-apt-get-install
#http://www.glom.org/wiki/index.php?title=Initial_Postgres_Configuration
#http://stackoverflow.com/questions/1471571/how-to-configure-postgresql-for-the-first-time
#http://blog.chinaunix.net/uid-11582448-id-745478.html
#http://wenku.baidu.com/link?url=EKmHBs1WVcWXrCUJ6l4Ma0Jp7xgvTE8BC-ccbHkL-dWbjb3T3BXzW0ncDT1KGc4lRRUwTdQn8dlrtjYkSrPyTxJGY6l-oBGRo2UT5Y5Jp4a
#http://www.codeweblog.com/postgresql-windows%E4%B8%8B%E4%BA%8C%E8%BF%9B%E5%88%B6%E6%96%87%E4%BB%B6%dE5%AE%89%E8%A3%85/