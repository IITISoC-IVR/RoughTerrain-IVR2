# Autonomous Navigation for Vehicles in Rough Terrain
<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/github_username/repo_name">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

![Prius Image](https://devdiscourse.blob.core.windows.net/devnews/25_07_2022_21_23_03_3131741.jpg)
<h3 align="center">project_title</h3>

  <p align="center">
    project_description
    <br />
    <a href="https://github.com/github_username/repo_name"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/github_username/repo_name">View Demo</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues">Report Bug</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](data:image/jpeg;base64,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)

Here's a blank template to get started: To avoid retyping too much info. Do a search and replace with your text editor for the following: `github_username`, `repo_name`, `twitter_handle`, `linkedin_username`, `email_client`, `email`, `project_title`, `project_description`

<p align="right">(<a href="#readme-top">back to top</a>)</p>



### Built With

* [![Next][Next.js]][Next-url]
* [![React][React.js]][React-url]
* [![Vue][Vue.js]][Vue-url]
* [![Angular][Angular.io]][Angular-url]
* [![Svelte][Svelte.dev]][Svelte-url]
* [![Laravel][Laravel.com]][Laravel-url]
* [![Bootstrap][Bootstrap.com]][Bootstrap-url]
* [![JQuery][JQuery.com]][JQuery-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* npm
  ```sh
  npm install npm@latest -g
  ```

### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   git clone https://github.com/github_username/repo_name.git
   ```
3. Install NPM packages
   ```sh
   npm install
   ```
4. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

_For more examples, please refer to the [Documentation](https://example.com)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

- [ ] Feature 1
- [ ] Feature 2
- [ ] Feature 3
    - [ ] Nested Feature

See the [open issues](https://github.com/github_username/repo_name/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Your Name - [@twitter_handle](https://twitter.com/twitter_handle) - email@email_client.com

Project Link: [https://github.com/github_username/repo_name](https://github.com/github_username/repo_name)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* []()
* []()
* []()

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo_name.svg?style=for-the-badge
[contributors-url]: https://github.com/github_username/repo_name/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo_name.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/repo_name/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo_name.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/repo_name/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo_name.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/repo_name/issues
[license-shield]: https://img.shields.io/github/license/github_username/repo_name.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo_name/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 
