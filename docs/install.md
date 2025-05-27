# Installation Guide

(Date: 27.05.2025)

## 1. Docker Installation Documentation:

You can follow the official Docker installation documentation for Ubuntu by visiting the following link:
[Docker Install Guide for Ubuntu](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

### Step-by-Step Installation:

1. Update your package list:

   ```bash
   sudo apt-get update
   ```

2. Install the required packages:

   ```bash
   sudo apt-get install ca-certificates curl
   ```

3. Add Docker's official GPG key:

   ```bash
   sudo install -m 0755 -d /etc/apt/keyrings
   sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
   sudo chmod a+r /etc/apt/keyrings/docker.asc
   ```

4. Add the Docker repository to your Apt sources:

   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   ```

5. Update your package list again:

   ```bash
   sudo apt-get update
   ```

6. Install Docker:

   ```bash
   sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   ```

7. Verify Docker installation by running the hello-world container:

   ```bash
   sudo docker run hello-world
   ```

## 2. Docker Post-Installation Steps:

After installing Docker, you can follow these post-installation steps:
[Docker Post-Install Guide](https://docs.docker.com/engine/install/linux-postinstall/)

1. Create the Docker group:

   ```bash
   sudo groupadd docker
   ```

2. Add your user to the Docker group:

   ```bash
   sudo usermod -aG docker $USER
   ```

3. Log out and log back in for the changes to take effect.

4. Start a new shell session:

   ```bash
   newgrp docker
   ```

5. Verify Docker is working without `sudo`:

   ```bash
   docker run hello-world
   ```

6. Enable Docker services to start on boot:

   ```bash
   sudo systemctl enable docker.service
   sudo systemctl enable containerd.service
   ```

---

## 3. Nvidia Container Toolkit

Full documentation: [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

Add the NVIDIA container toolkit repository:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Update packages and install the toolkit:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
```

Then restart your computer.

## 4. Docker Compose Startup

Install the following VS Code extensions:

* **Remote Development**
* **Docker**

In your project directory:

```bash
cd docker

docker compose up explorer_container
```

## 5. Open the Dev Container in VS Code

1. Open VS Code
2. Go to the **Docker tab**
3. Locate the container (e.g. `ros2/explorer`)
4. Right-click and select **Attach to Container**
5. A new VS Code window will open inside the container
6. Open the terminal inside the container
7. Navigate to the `/app` folder and open it in VS Code
