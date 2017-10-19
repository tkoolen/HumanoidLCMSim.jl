using StaticArrays

import LCMCore: decode, encode

const NETWORK_BYTE_ORDER_TYPES = Union{Int8, Int16, Int32, Int64, Float32, Float64, UInt8}

struct FingerprintException <: Exception
    message::String
end

function fingerprint end

decode!(::T, io::IOBuffer) where {T} = decode(io, T)

function decode!(out, data::Vector{UInt8})
    io = IOBuffer(data)
    if read(io, SVector{8, UInt8}) != fingerprint(typeof(out))
        throw(FingerprintException("LCM message fingerprint did not match. This means that you are trying to decode the wrong message type, or a different version of the message type."))
    end
    decode!(out, io)
end

function decode!(out::AbstractVector{T}, io::IOBuffer) where T
    for i in eachindex(out)
        out[i] = decode!(out[i], io)
    end
    out
end

decode(io::IOBuffer, ::Type{T}) where {T <: NETWORK_BYTE_ORDER_TYPES} = ntoh(read(io, T))

function decode(io::IOBuffer, ::Type{String})
    len = ntoh(read(io, UInt32))
    ret = String(read(io, len - 1))
    read(io, UInt8) # strip off null
    ret
end
