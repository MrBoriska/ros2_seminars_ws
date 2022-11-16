
# Deploy and update with ansible

Inventory (/etc/ansible/hosts)

```ini
[students]
host0.example.org
host1.example.org
host2.example.org
```

delete old repo

```bash
ansible -i students_admin -m shell -a 'sudo rm -R ./home/student/ros2_seminars_ws' students
```

add new

```bash
ansible -i students_user -m copy -a 'src=/home/student/ros2_seminars_ws dest=/home/student/ros2_seminars_ws' students
```

install deps

```bash
ansible -i students_admin -m shell -a './home/student/ros2_seminars_ws/install_ros2_humble_deps.sh' students
```

with sudo use this example

```bash
ansible -i students_admin -b --ask-become-pass -m shell -a './home/student/ros2_seminars_ws/install_ros2_humble_deps.sh' students
```


ansible -i students_hosts -m shell -a 'DISPLAY=:0 firefox --url https://github.com/MrBoriska/ros2_seminars_ws/blob/master/%D0%A1%D0%B5%D0%BC%D0%B8%D0%BD%D0%B0%D1%80%201.md' students
