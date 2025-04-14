function clamped_val = Clamp(val, min, max)
clamped_val = val;
if(clamped_val > max)
  clamped_val = max;
elseif(clamped_val < min)
  clamped_val = min;
end
  
endfunction
