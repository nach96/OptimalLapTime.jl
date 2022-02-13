module OptimalLapTime
export goodbye

""" A greet function
# Arguments
- None
"""
function greet()
    str = "Hello World"
    print(str)
    return str;
end

"""A goodbye function
"""
goodbye(who) = print("Bye "*who)

end # module