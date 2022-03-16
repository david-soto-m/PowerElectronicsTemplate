%% Start data
pl=plot_class();
sec_str = "Segundos";
%% big
deg_change = 30;
out=sim("parte_1.slx");
datas={
    out.P_alphabet,"Potencia \alpha \beta", {"Watios", sec_str},[], [];
    out.V_alphabet,"Tensión \alpha \beta", {"Voltios", sec_str},[], [];
    out.I_alphabet,"Intensidad \alpha \beta", {"Amperios", sec_str},[], [];
};
pl.deal_datas(datas, [3,1], "alphabet_inc");

close all;
