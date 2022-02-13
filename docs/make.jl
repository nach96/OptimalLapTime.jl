using Documenter
using OptimalLapTime

makedocs(
    sitename = "OptimalLapTime",
    #format = Documenter.HTML(),
    modules = [OptimalLapTime]
)

# Documenter can also automatically deploy documentation to gh-pages.
# See "Hosting Documentation" and deploydocs() in the Documenter manual
# for more information.
#=deploydocs(
    repo = "<repository url>"
)=#
deploydocs(
    repo = "github.com/nach96/OptimalLapTime.jl.git",
)
