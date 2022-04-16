"""
debug print.
# Arguments
- vb [boolean] Print only if true.
- xs... Comma separated arguments to print
# example
dprint(true,"Hey, I am ",25)
"""
function dprint(vb,xs...)
    if vb print(xs...) end
end


dprintln(vb,xs...) = dprint(vb,xs...,'\n')
