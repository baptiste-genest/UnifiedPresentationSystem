#include "prompter.h"
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

UPS::Prompter::Prompter(std::string script_file) : script_file(script_file)
{

    prompt_pid = fork();

    if (prompt_pid == -1) {
        std::cerr << "[couldn't fork to prompt]" << std::endl;
        assert(false);
    }

    if (prompt_pid == 0) {
        system("touch /tmp/prompt.txt");
        //execlp("gnome-terminal", "UPS_PROMPT", "--", "watch", "-n", "1", "cat", "/tmp/prompt.txt", NULL);
        system("xterm -fg white -bg black -fa 'Monospace' -fs 20 -e \"watch -n 0.5 cat /tmp/prompt.txt\"");
        //execlp("xterm", "xterm_ups_prompt", "-e ", "\"watch", "-n","1","cat","/tmp/prompt.txt\"",NULL);

        // Si execlp échoue, afficher une erreur
        //std::cerr << "[Error while starting prompt]" << std::endl;
        abort();
        //assert(false);
    }         //system("gnome-terminal -- watch -n 1 cat /tmp/prompt.txt");
    /*
    else
        std::cout << "PROMPT PID " << prompt_pid << std::endl;
        */

}

void UPS::Prompter::write(promptTag tag) const
{
    if (!scripts.contains(tag)){
        std::cerr << "[INVALID TAG] " << tag << std::endl;
        assert(0);
    }
    std::ofstream file("/tmp/prompt.txt");
    file << "|----------------------------------------------------------------|" << std::endl;
    file << "|                           UPS PROMPT                           |" << std::endl;
    file << "|----------------------------------------------------------------|" << std::endl;
    file << scripts.at(tag);
    file.close();
}

void UPS::Prompter::loadScript()
{
    std::ifstream script(script_file);
    std::string line;
    while (std::getline(script,line))
    {
        if (line.empty())
            continue;
        if (line[0] == '['){
            line.erase(remove(line.begin(), line.end(), '['), line.end());
            line.erase(remove(line.begin(), line.end(), ']'), line.end());
            current_tag = line;
        }
        else if (!line.empty() && current_tag.empty()) {
            std::cerr << " [INVALID SCRIPT FORMAT, MUST INSERT \"[TAG]\" BEFORE TEXT ]" << std::endl;
        }
        else {
            if (!scripts.contains(current_tag))
                scripts[current_tag] = "";
            scripts[current_tag] += line + "\n";
        }
    }
}
