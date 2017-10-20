using StaticArrays

import LCMCore: decode, encode

const NETWORK_BYTE_ORDER_TYPES = Union{Int8, Int16, Int32, Int64, Float32, Float64, UInt8}

abstract type LCMType end

# Checks
"""
    check_valid(x::LCMType)

Check that `x` is a valid LCM type. For example, check that array lengths are correct.
"""
check_valid(x::LCMType) = nothing # TODO: generate from LCM type specification

"""
    fingerprint(<:LCMType)

Return the fingerprint of the LCM type.
"""
fingerprint(::T) where {T <: LCMType} = fingerprint(T) # TODO: generate from type

struct FingerprintException <: Exception
    T::Type
end

@noinline function Base.showerror(io::IO, e::FingerprintException)
    print(io, "LCM message fingerprint did not match type ", e.T, ". ")
    print(io, "This means that you are trying to decode the wrong message type, or a different version of the message type.")
end

function check_fingerprint(io::IO, ::Type{T}) where T <: LCMType
    if read(io, SVector{8, UInt8}) != fingerprint(T)
        throw(FingerprintException(T))
    end
end


## Decoding
"""
    decode!(x, io::IO)

Decode bytes from `io` into an object of type `typeof(x)`. `x` may be used to improve performance,
but regardless of whether `x` is modified, methods must return the decoded object.

The default behavior is to call `decode(io, typeof(x))`.
"""
function decode!(x, io::IO) end
decode!(::T, io::IO) where {T} = decode(io, T)
decode!(x, data::Vector{UInt8}) = decode!(x, IOBuffer(data)) # TODO: consider removing

decode(io::IO, ::Type{T}) where {T <: NETWORK_BYTE_ORDER_TYPES} = ntoh(read(io, T))

function decode(io::IO, ::Type{String})
    len = ntoh(read(io, UInt32))
    ret = String(read(io, len - 1))
    read(io, UInt8) # strip off null
    ret
end

function decode!(x::AbstractVector{T}, io::IO) where T
    for i in eachindex(x)
        x[i] = decode!(x[i], io)
    end
    x
end


## Encoding
"""
    encode(io::IO, x::LCMType)

Write an LCM byte representation of `x` to `io`.
"""
@generated function encode(io::IO, x::LCMType)
    expr = Expr(:block)
    push!(expr.args, :(check_valid(x)))
    push!(expr.args, :(write(io, fingerprint(x))))
    for fieldname in fieldnames(x)
        push!(expr.args, :(encode(io, x.$fieldname)))
    end
    push!(expr.args, :(return io))
    expr
end

encode(io::IO, x::NETWORK_BYTE_ORDER_TYPES) = write(io, hton(x))

function encode(io::IO, x::String)
    write(io, hton(UInt32(length(x) + 1)))
    write(io, x)
    write(io, UInt8(0))
end

function encode(io::IO, A::AbstractVector)
    for x in A
        encode(io, x)
    end
end

encode(data::Vector{UInt8}, x::LCMType) = encode(IOBuffer(data), x) # TODO: consider removing
encode(x::LCMType) = encode(IOBuffer(), x).data # TODO: consider removing
