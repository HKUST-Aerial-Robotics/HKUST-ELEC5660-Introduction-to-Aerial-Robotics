terraform {
  required_providers {
    coder = {
      source = "coder/coder"
    }
    docker = {
      source = "kreuzwerker/docker"
    }
  }
}

variable "cpus" {
  description = "CPU limit for the workspace container."
  type        = number
  default     = 16
}

variable "memory_gb" {
  description = "Memory limit (GiB) for the workspace container."
  type        = number
  default     = 32
}

variable "shm_gb" {
  description = "Size of /dev/shm (GiB) for the workspace container."
  type        = number
  default     = 2
}

locals {
  username = lower(data.coder_workspace_owner.me.name)
  cpu_period   = 100000
  cpu_quota    = ceil(var.cpus * local.cpu_period)
  memory_mb    = floor(var.memory_gb * 1024)
  shm_bytes    = floor(var.shm_gb * 1024 * 1024 * 1024)
}

data "coder_provisioner" "me" {
}

provider "docker" {
}

data "coder_workspace" "me" {
}

data "coder_workspace_owner" "me" {
}

resource "coder_agent" "main" {
  arch           = data.coder_provisioner.me.arch
  os             = "linux"
  startup_script = <<-EOT
    set -e
    # Prepare user home with default files on first start.
    if [ ! -f ~/.init_done ]; then
      # Copy contents of /etc/skel to home directory (preserving existing files)
      cp -rn /etc/skel/. ~/
      touch ~/.init_done
    fi
  EOT

  display_apps {
    vscode = true
    ssh_helper = true
    port_forwarding_helper = true
    web_terminal = true
  }

  # These environment variables allow you to make Git commits right away after creating a
  # workspace. Note that they take precedence over configuration defined in ~/.gitconfig!
  # You can remove this block if you'd prefer to configure Git manually or using
  # dotfiles. (see docs/dotfiles.md)
  env = {
    GIT_AUTHOR_NAME     = coalesce(data.coder_workspace_owner.me.full_name, data.coder_workspace_owner.me.name)
    GIT_AUTHOR_EMAIL    = "${data.coder_workspace_owner.me.email}"
    GIT_COMMITTER_NAME  = coalesce(data.coder_workspace_owner.me.full_name, data.coder_workspace_owner.me.name)
    GIT_COMMITTER_EMAIL = "${data.coder_workspace_owner.me.email}"
  }

  # The following metadata blocks are optional. They are used to display
  # information about your workspace in the dashboard. You can remove them
  # if you don't want to display any information.
  # For basic resources, you can use the `coder stat` command.
  # If you need more control, you can write your own script.
  metadata {
    display_name = "CPU Usage"
    key          = "0_cpu_usage"
    script       = "coder stat cpu"
    interval     = 10
    timeout      = 1
  }

  metadata {
    display_name = "RAM Usage"
    key          = "1_ram_usage"
    script       = "coder stat mem"
    interval     = 10
    timeout      = 1
  }

  metadata {
    display_name = "Home Disk"
    key          = "3_home_disk"
    script       = "coder stat disk --path $${HOME}"
    interval     = 60
    timeout      = 1
  }

  metadata {
    display_name = "CPU Usage (Host)"
    key          = "4_cpu_usage_host"
    script       = "coder stat cpu --host"
    interval     = 10
    timeout      = 1
  }

  metadata {
    display_name = "Memory Usage (Host)"
    key          = "5_mem_usage_host"
    script       = "coder stat mem --host"
    interval     = 10
    timeout      = 1
  }

  metadata {
    display_name = "Load Average (Host)"
    key          = "6_load_host"
    # get load avg scaled by number of cores
    script   = <<EOT
      echo "`cat /proc/loadavg | awk '{ print $1 }'` `nproc`" | awk '{ printf "%0.2f", $1/$2 }'
    EOT
    interval = 60
    timeout  = 1
  }

  metadata {
    display_name = "Swap Usage (Host)"
    key          = "7_swap_host"
    script       = <<EOT
      free -b | awk '/^Swap/ { printf("%.1f/%.1f", $3/1024.0/1024.0/1024.0, $2/1024.0/1024.0/1024.0) }'
    EOT
    interval     = 10
    timeout      = 1
  }
}

resource "coder_app" "novnc" {
  agent_id     = coder_agent.main.id
  slug         = "novnc"
  display_name = "noVNC"
  # Use vnc_lite.html which has simpler path handling
  # The path parameter must NOT have a leading slash for noVNC to work correctly
  url          = "http://localhost:6080?&path=@${data.coder_workspace_owner.me.name}/${data.coder_workspace.me.name}.main/apps/novnc/websockify"
  icon         = "https://dashboard.snapcraft.io/site_media/appmedia/2020/07/novnc-icon.svg.png"
  share        = "owner"
  subdomain    = false

  healthcheck {
    url       = "http://localhost:6080"
    interval  = 5
    threshold = 10
  }
}

resource "docker_container" "workspace" {
  count = data.coder_workspace.me.start_count
  image = "hkustswarm/elec5660:2026"
  # Uses lower() to avoid Docker restriction on container names.
  name = "coder-${data.coder_workspace_owner.me.name}-${lower(data.coder_workspace.me.name)}"
  # Hostname makes the shell more user friendly: coder@my-workspace:~$
  hostname = data.coder_workspace.me.name

  cpu_period = local.cpu_period
  cpu_quota  = local.cpu_quota
  memory     = local.memory_mb
  shm_size   = local.shm_bytes

  # Use the docker gateway if the access URL is 127.0.0.1
  entrypoint = [
    "sh", "-c",
    <<-EOT
    set -x
    # Create directory structure in shared volume
    USER_WORKSPACE_DIR="/coder-home/${local.username}/${data.coder_workspace.me.name}"
    mkdir -p "$USER_WORKSPACE_DIR"

    # Create user logic
    if ! id -u "${local.username}" >/dev/null 2>&1; then
      # Create user without home directory first
      useradd -M -s /bin/bash "${local.username}"
    fi

    # Set the user's home directory to /home/<user>
    usermod -d "/home/${local.username}" "${local.username}"

    # Add user to video and render groups for GPU access
    groupadd -f video
    groupadd -f render

    # Dynamically add user to the groups that own /dev/dri devices
    if [ -d /dev/dri ]; then
      # Get unique group IDs from /dev/dri devices
      for gid in $(stat -c '%g' /dev/dri/* 2>/dev/null | sort -u); do
        # Get or create group name for this GID
        group_name=$(getent group $gid | cut -d: -f1)
        if [ -n "$group_name" ]; then
          usermod -a -G "$group_name" "${local.username}" 2>/dev/null || true
        fi
      done
    fi

    # Ensure user is in video and render groups
    usermod -a -G video,render "${local.username}"

    # Create /home/<user> as symlink to shared workspace dir
    if [ -e "/home/${local.username}" ] && [ ! -L "/home/${local.username}" ]; then
      rm -rf "/home/${local.username}"
    fi
    ln -sfn "$USER_WORKSPACE_DIR" "/home/${local.username}"

    # Ensure sudo access
    echo "${local.username} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/coder
    chmod 0440 /etc/sudoers.d/coder

    # Fix ownership of home directory (crucial for mounted volumes)
    chown -R "${local.username}:${local.username}" "$USER_WORKSPACE_DIR"

    # Prepare Coder agent init script
    cat <<'EOAG' > /tmp/coder-init
    ${replace(coder_agent.main.init_script, "/localhost|127\\.0\\.0\\.1/", "host.docker.internal")}
EOAG
    chmod +x /tmp/coder-init

    # Run the agent as the user
    # We use '&' and 'wait' so that if it exits, we can catch it or ignore it
    # We must explicitly export the token because 'su -' clears environment variables
    su - "${local.username}" -c "export CODER_AGENT_TOKEN=$${CODER_AGENT_TOKEN}; /tmp/coder-init" &

    # Start supervisor if available
    if [ -f /system/supervisor.sh ]; then
      # Update HOME for XFCE session
      sed -i "s|HOME=\"/\"|HOME=\"$USER_WORKSPACE_DIR\"|g" /system/supervisord.conf

      # Ensure log files exist and are writable by the user
      touch /var/log/xfce4.log /var/log/xfce4.err \
            /var/log/xvfb.log /var/log/xvfb.err \
            /var/log/x11vnc.log /var/log/x11vnc.err
      chown ${local.username}:${local.username} /var/log/xfce4.* /var/log/xvfb.* /var/log/x11vnc.*

      # Run supervisor as the user
      su - "${local.username}" -c "/system/supervisor.sh" &
    fi

    # Wait for the agent. If it exits, we just sleep to keep container debugging possible
    wait $! || true
    EOT
  ]
  runtime    = "nvidia"
  gpus       = "device=0"
  env        = ["CODER_AGENT_TOKEN=${coder_agent.main.token}"]
  host {
    host = "host.docker.internal"
    ip   = "host-gateway"
  }
  devices {
    host_path      = "/dev/dri"
    container_path = "/dev/dri"
    permissions    = "rwm"
  }
  volumes {
    container_path = "/coder-home"
    volume_name    = "coder-service_coder_home"
    read_only      = false
  }

  # Add labels in Docker to keep track of orphan resources.
  labels {
    label = "coder.owner"
    value = data.coder_workspace_owner.me.name
  }
  labels {
    label = "coder.owner_id"
    value = data.coder_workspace_owner.me.id
  }
  labels {
    label = "coder.workspace_id"
    value = data.coder_workspace.me.id
  }
  labels {
    label = "coder.workspace_name"
    value = data.coder_workspace.me.name
  }
}
