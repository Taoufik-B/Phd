https://tmuxcheatsheet.com/

## Aliases

```bash
# >>> Aliases for phd project access
alias phd="cd /home/taoufik/projects/Phd--POMDP"
alias c14="/home/taoufik/software/CARLA_0.9.14/CarlaUE4.sh -windowed -ResX=800 -ResY=600 -carla-server -quality-level=Low"
alias c15="/home/taoufik/software/CARLA_0.9.15/CarlaUE4.sh -windowed -ResX=800 -ResY=600 -carla-server -quality-level=Low"
```


# Linux commands

- sed
- awk
- jq

#### ip with jq

`ip -4 -j -br a | jq '[.[]|{ifname: .ifname, ip: .addr_info[].local}]'`

## Jupyter lab

`conda activate py37`
`jupyter-lab`

