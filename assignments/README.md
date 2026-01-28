# Environment Setup

This course primarily uses Python and C++. Most labs and assignments are built around ROS 1.

## Self-Hosted Environment (Recommended)

See `workspace_template/ELEC5660/` for the environment definition. You can:

- Build and run the provided Docker image (recommended), or
- Install dependencies locally by following the Dockerfile. This method only supports Ubuntu 20.04 & 18.04.

### Using Docker Locally

**Prerequisites:**

- [Docker](https://docs.docker.com/get-docker/)

**Build the Docker image:**

```bash
cd workspace_template/ELEC5660/docker
./build.sh
```

**Run the Docker container:**

`-v` mounts the assignment code into the container at `/ws`， you can modify it as needed.

```bash
./run.sh
```

**Access the Dev Environment with VSCode:**

Required Extensions:

| Extension | ScreenShot |
| --- | --- |
| Container Tools | ![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260128161532475.png) |
| Remote Development | ![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240827175422.png) |
| C/C++ Extension Pack | ![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240827175144.png) |

Right-click on the container and click `Attach Visual Studio Code`.

![image-20260128161733580](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260128161733580.png)

Then click `File -> Open Folder` on the newly popped up VSCode window and select the workspace folder `/ws`.

![image-20260128161838730](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260128161838730.png)

**Setting up GUI applications (e.g., RViz, Gazebo)**

- If you have X server running on your host machine (most of linux desktop environments), you can run GUI applications inside the Docker container by allowing access to the X server:

  *On the host machine, run:*
  ```bash
  # Find your DISPLAY variable
  echo $DISPLAY # e.g., :0
  # Allow access to X server
  xhost +
  ```

  *Inside the Docker container, set the DISPLAY variable:*
  ```bash
  export DISPLAY=:0 # Use the same value as on the host machine
  ```

- If there is no X server on your host machine, you can access `localhost:6080` on your host machine browser to use noVNC for GUI applications inside the Docker container, and the default `DISPLAY` variable is `:10`.

## Cloud Environment (An application is required)

**Only when you don't have the appropriate equipment or encounter difficulties in establishing a local environment, you may apply for a cloud coding environment.**

We prepare a cloud coding environment hosted on [Coder for HKUST-UAV](https://code.hkust-uav.org), which is based on [Coder](https://coder.com/). This environment comes pre-installed with all necessary dependencies for the assignments and labs.

### Account

By applying for a cloud coding environment, you will receive an account with the following credentials:

- Platform: https://code.hkust-uav.org
- Username: your ITSC email (e.g., `yxuew@connect.ust.hk`)
- Initial password: your ITSC email (same as above)

Please change your password after the first login.

![image-20260126172127683](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260126172127683.png)

### Create a workspace

After logging in, create a new workspace using the `ELEC5660-2026-Env` template.

Once created, you can connect via:

- VS Code Desktop
- noVNC (remote desktop for GUI)
- Terminal

![image-20260126172700354](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260126172700354.png)

### Coding with VSCode & Visualization with noVNC

![image-20260126174951584](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260126174951584.png)

![image-20260126174727437](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260126174727437.png)

### Notes

- Workspaces stop automatically after 24 hours by default. You can disable **Autostop** under the workspace **Schedule** settings.
- Only `/home/<user>/` is persistent. All other data may be lost after a stop/restart (e.g., packages installed via `apt`).