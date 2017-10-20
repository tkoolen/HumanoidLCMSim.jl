using StaticArrays

import LCMCore: decode, encode

abstract type LCMType end

const NETWORK_BYTE_ORDER_TYPES = Union{Int8, Int16, Int32, Int64, Float32, Float64, UInt8}

struct FingerprintException <: Exception
    message::String
end

fingerprint(::T) where {T} = fingerprint(T)

function check_fingerprint(io::IO, ::Type{T}) where T
    if read(io, SVector{8, UInt8}) != fingerprint(T)
        throw(FingerprintException("LCM message fingerprint did not match. This means that you are trying to decode the wrong message type, or a different version of the message type."))
    end
end


# Decoding
decode!(::T, io::IO) where {T} = decode(io, T)
decode!(out, data::Vector{UInt8}) = decode!(out, IOBuffer(data))

decode(io::IO, ::Type{T}) where {T <: NETWORK_BYTE_ORDER_TYPES} = ntoh(read(io, T))

function decode(io::IO, ::Type{String})
    len = ntoh(read(io, UInt32))
    ret = String(read(io, len - 1))
    read(io, UInt8) # strip off null
    ret
end

function decode!(out::AbstractVector{T}, io::IO) where T
    for i in eachindex(out)
        out[i] = decode!(out[i], io)
    end
    out
end


# Encoding
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

@generated function encode(io::IO, x::LCMType)
    expr = Expr(:block)
    push!(expr.args, :(write(io, fingerprint(x))))
    for fieldname in fieldnames(x)
        push!(expr.args, :(encode(io, x.$fieldname)))
    end
    push!(expr.args, :(return io))
    expr
end

encode(data::Vector{UInt8}, x::LCMType) = encode(IOBuffer(data), x)
encode(x::LCMType) = encode(IOBuffer(), x).data
