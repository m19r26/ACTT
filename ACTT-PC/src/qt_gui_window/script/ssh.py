#!/usr/bin/env python
# coding: UTF-8

import paramiko

client_actt = paramiko.SSHClient()
#client_actt.set_missing_host_key_policy(paramiko.WarningPolicy())
client_actt.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client_actt.connect('192.168.8.77', username='pi', password='robOITics')

stdin, stdout, stderr = client_actt.exec_command('source ~/opencampas/oc.sh')

for o in stderr:
   print(o)
    
client_actt.close()
