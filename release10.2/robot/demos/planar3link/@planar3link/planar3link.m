classdef planar3link < SerialLink
 
	properties
	end
 
	methods
		function ro = planar3link()
			objdir = which('planar3link');
			idx = find(objdir == filesep,2,'last');
			objdir = objdir(1:idx(1));
			 
			tmp = load(fullfile(objdir,'@planar3link','matplanar3link.mat'));
			 
			ro = ro@SerialLink(tmp.sr);
			 
			 
		end
	end
	 
end
