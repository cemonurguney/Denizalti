classdef reference_motion < handle

    properties
        reference
    end

    methods

        function edit(obj,shape,long)
            if shape =="line"
                obj.reference = [ ...
                     long;
                     long;
                     0];
            end


            if shape == "kup"

                obj.reference = [ ...
                     long   long  -long  -long   long   long   long   long -long  -long   long   long  -long  -long  -long  -long  -long;
                    -long   long   long  -long  -long  -long   long   long   long   long   long  -long  -long   long   long  -long  -long;
                      0    0    0    0    0  long*2  long*2    0    0  long*2  long*2  long*2  long*2  long*2    0    0  long*2];

            elseif shape == "kare"

                obj.reference = [ ...
                     long   long  -long  -long   long;
                    -long   long   long  -long  -long;
                      0    0    0    0    0];

            end

        end

    end
end