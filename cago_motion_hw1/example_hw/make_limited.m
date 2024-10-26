function a=make_limited(f)
a=f;
if(f>=360)
    a=f-360;
elseif(f<0)
    a=f+360;
end
    