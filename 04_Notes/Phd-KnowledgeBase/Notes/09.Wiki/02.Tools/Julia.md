# Installation

[Reference](https://medium.com/coffee-in-a-klein-bottle/install-julia-1-5-on-ubuntu-bb8be4b2571d)

```sh
tar -xvzf julia-1.6.4-linux-x86_64.tar.gz
sudo mv julia-1.6.4/ /opt/
sudo ln -s /opt/julia-1.6.4/bin/julia /usr/local/bin/julia
julia> using Pkg
julia> Pkg.add(“IJulia”)
julia> using IJulia
julia> installkernel("Julia")
```


```sh
curl -fsSL https://install.julialang.org | sh
juliaup list
```
