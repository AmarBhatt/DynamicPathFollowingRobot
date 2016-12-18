function dev=connect_bluetooth
g=msgbox('scanning bluetooth devices');
pause(.1);
if(ishandle(g))
delete(g);
end
r=instrhwinfo('Bluetooth');
d=r.RemoteNames;
[m,n] = listdlg('PromptString','Select Your Bluetooth device:','SelectionMode','single','ListString',d);
if(n==1)
dn=cell2mat(d(m));
di=instrhwinfo('Bluetooth',dn);
if(~isempty(cell2mat(di.Channels)))
dev= Bluetooth(dn,str2double(di.Channels));
fopen(dev);
else
msgbox('Not able to connect, Restart Device And Connect Again');
end
else
msgbox('Atleast one bluetooth Device Must be connected');
end
end
