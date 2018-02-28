module NotebookTests
using NBInclude

@testset "notebooks" begin
    notebookdir = joinpath("..", "notebooks")
    for file in readdir(notebookdir)
        name, ext = splitext(file)
        if lowercase(ext) == ".ipynb"
            @testset "$name" begin
                println("Testing $name.")
                nbinclude(joinpath(notebookdir, file), regex = r"^((?!\#NBSKIP).)*$"s)
            end
        end
    end
end

end # module
