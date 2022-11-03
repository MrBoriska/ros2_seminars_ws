
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
ansible -u root -m shell -a 'sudo rm -R ./home/student/ros2_seminars_ws' students
```

add new

```bash
ansible -u student -m copy -a 'src=/home/student/ros2_seminars_ws dest=/home/student/ros2_seminars_ws' students
```

install deps and build repo

```bash
ansible -u student -m shell -a 'sudo ./home/student/ros2_seminars_ws/install_ros2_humble_deps.sh' students
```