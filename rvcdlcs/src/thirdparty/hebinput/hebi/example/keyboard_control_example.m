% use 'HebiKeyboard' in https://github.com/HebiRobotics/MatlabInput
close all
clear

kb = HebiKeyboard;
while true
   state = read(kb);
   % number 0-9, letter A-Z or a-z, symbol []\;
   letter = find(state.keys('0':']'))+'0'-1;
   if ~isempty(letter)
       disp([char(letter) ' are pressed!']);
   end
   if state.UP
       disp('up is pressed!')
   end
   pause(0.1)
end

% kb = pkgHebi.ikjlHebiKeyboard();
% while true
%     state = read(kb);
%     state.keys('ikjl')
%     if all(state.keys('x0'))
%         disp('x and 0 are both pressed!')
%     end
%     pause(0.01);
% end
% 
% kb = HebiKeyboard();
% while true
%     state = read(kb);
%     down = find(state.keys('a':'z')) + 'a'-1;
%     if ~state.SHIFT
%         disp(char(down));
%     end
%     pause(0.01);
% end