function range_to_ind(range)
    length(range) == 1 || throw(ArgumentError("length(range) != 1"))
    first(range)
end
