# Развертывание и управление учебным классом с помощью Ansible

Данная директория содержит конфигурационные файлы, инвентари и сценарии для централизованного развертывания, обновления и обслуживания учебного окружения ROS 2 Humble в компьютерном классе на $N$ машинах.

---

## 1. Структура каталога `deploy/`

| Файл | Назначение |
| :--- | :--- |
| `students_admin` | Инвентарь Ansible для учетных записей администраторов компьютеров (доступ с `sudo`) |
| `students_user` | Инвентарь Ansible для стандартной учетной записи студента (`student`) |
| `students_admin2` | Дополнительный инвентарь для альтернативных хостов/пользователей |
| `install_ros2_humble_deps.sh` | Скрипт пакетной установки всех системных зависимостей ROS 2 Humble, Gazebo и сопутствующих пакетов |
| `setup_workspace.sh` | Скрипт конфигурации окружения `.bashrc` (автоопределение `ROS_DOMAIN_ID`, `source /opt/ros/...`) |
| `install_gstreamer.yaml` | Playbook для установки библиотек GStreamer |
| `install_vino.yaml` | Playbook для настройки сервера удаленного рабочего стола |

---

## 2. Подготовка перед началом работы

### 2.1. Генерация и копирование SSH-ключей преподавателя

Чтобы выполнять команды на всех машинах без постоянного ввода паролей:
```bash
# Генерация ключа (если еще не создан)
ssh-keygen -t ed25519 -C "teacher@lab"

# Копирование ключа на машины студентов
for ip in 172.23.9.{160..173}; do
    ssh-copy-id rim-$ip@$ip
done
```

### 2.2. Отключение проверки Host Key (для удобства работы в изолированной сети)

В `~/.ansible.cfg` или в локальном `ansible.cfg`:
```ini
[defaults]
host_key_checking = False
timeout = 30
```

---

## 3. Сценарии типовых операций

### 3.1. Проверка доступности машин (Ping)
```bash
ansible -i students_admin -m ping students
```

### 3.2. Удаление старой версии репозитория у студентов
```bash
ansible -i students_admin -b -m shell -a 'rm -rf /home/student/ros2_seminars_ws' students
```

### 3.3. Клонирование или копирование актуального репозитория

**Вариант А: Клонирование из Git прямо на машинах:**
```bash
ansible -i students_user -m git -a 'repo=https://github.com/MrBoriska/ros2_seminars_ws.git dest=/home/student/ros2_seminars_ws version=master' students
```

**Вариант Б: Прямое копирование с ПК преподавателя:**
```bash
ansible -i students_user -m copy -a 'src=../../ros2_seminars_ws/ dest=/home/student/ros2_seminars_ws/' students
```

### 3.4. Установка зависимостей ROS 2 Humble и Gazebo
Выполняется от имени администратора с правами `sudo`:
```bash
ansible -i students_admin -b --ask-become-pass -m shell -a '/home/student/ros2_seminars_ws/deploy/install_ros2_humble_deps.sh' students
```

### 3.5. Настройка окружения и изоляции сети (`ROS_DOMAIN_ID`)
Для того чтобы студенты не "перебивали" топики и ноды друг друга в одной локальной сети, на каждой машине настраивается уникальный `ROS_DOMAIN_ID`:
```bash
ansible -i students_user -m shell -a '/home/student/ros2_seminars_ws/deploy/setup_workspace.sh' students
```

### 3.6. Сборка воркспейса (`colcon build`)
```bash
ansible -i students_user -m shell -a "chdir='/home/student/ros2_seminars_ws' cmd='source /opt/ros/humble/setup.bash && colcon build --symlink-install'" students
```

### 3.7. Запуск браузера с заданием семинара на экране студента
Полезно для синхронного открытия нужного задания на всех мониторах в аудитории:
```bash
ansible -i students_user -m shell -a 'DISPLAY=:0 xdg-open https://github.com/MrBoriska/ros2_seminars_ws/blob/master/%D0%A1%D0%B5%D0%BC%D0%B8%D0%BD%D0%B0%D1%80%201.md' students
```

---

## 4. Примечания к Playbook'ам

### `install_vino.yaml`
> [!NOTE]
> В Ubuntu 22.04 LTS окружение рабочего стола GNOME 42 по умолчанию использует сервер `gnome-remote-desktop` (протокол RDP / VNC). Пакет `vino` работает только в устаревших X11-сессиях. При использовании современных дистрибутивов рекомендуется включать удаленный доступ через Настройки GNOME (`Sharing -> Remote Desktop`) или устанавливать пакет `gnome-remote-desktop`.
