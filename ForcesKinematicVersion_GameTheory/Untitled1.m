for ii=1:length(outputM(:,index_IBR.v))
    ab(ii)=outputM(ii,index_IBR.ab)-casadiGetSmoothMaxAcc(outputM(ii,index_IBR.v));
end