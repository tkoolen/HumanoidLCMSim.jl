module Sides

using Random

export
    Side,
    left,
    right,
    flipsign_if_right

@enum Side left right
flipsign_if_right(x::Number, side::Side) = ifelse(side == right, -x, x)
Base.rand(rng::AbstractRNG, ::Type{Side}) = ifelse(rand(rng, Bool), left, right)
Base.:(-)(side::Side) = ifelse(side == right, left, right)

end # module
