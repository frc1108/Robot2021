import paramiko

policy = paramiko.client.AutoAddPolicy

with paramiko.SSHClient () as client:
    client.set_missing_host_key_policy

